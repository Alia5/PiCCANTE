/*
 * PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
 * Copyright (C) 2025 Peter Repukat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "emulator.hpp"
#include <string_view>
#include <functional>
#include <cctype>
#include "at_commands/commands.hpp"
#include "CanBus/CanBus.hpp"
extern "C" {
#include <can2040.h>
}
#include <pico/time.h>
#include "Logger/Logger.hpp"
#include "util/hexstr.hpp"
#include "at_commands/commands.hpp"

namespace piccante::elm327 {

bool emulator::start() {
    if (running) {
        return true;
    }


    running = true;
    BaseType_t res = xTaskCreate(emulator_task,
                                 "ELM327 Emu",
                                 configMINIMAL_STACK_SIZE,
                                 this,
                                 tskIDLE_PRIORITY + 5,
                                 &task_handle);

    if (res != pdPASS) {
        running = false;
        Log::error << "ELM327: Failed to create task: " << res << "\n";
        return false;
    }
    Log::info << "ELM327: Emulator started\n";
    return true;
}

void emulator::stop() {
    if (!running) {
        return;
    }
    if (task_handle) {
        for (int i = 0; i < 10 && eTaskGetState(task_handle) != eDeleted; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (eTaskGetState(task_handle) != eDeleted) {
            vTaskDelete(task_handle);
        }
        task_handle = nullptr;
    }
}

bool emulator::is_running() const { return running; }

void emulator::handle(std::string_view command) {
    if (command.empty()) {
        return;
    }

    if (command[0] == '\r' || command[0] == '\n') {
        handle(last_command);
        return;
    }
    last_command = command;

    if (is_valid_hex(command)) {
        // assume every only hex command is a valid pid request...
        handle_pid_request(command);
        return;
    }
    Log::debug << "ELM327: command: " << command << "\n";
    // TODO: check if command is valid.
    handle_command(command);
}
void emulator::handle_command(std::string_view command) {
    if (at_h.is_at_command(command)) {
        at_h.handle(command);
    }
}

void emulator::handle_pid_request(std::string_view command) {
    can2040_msg frame{};

    frame.id = params.obd_header;
    if (params.use_extended_frames) {
        frame.id |= CAN2040_ID_EFF;
    }

    frame.dlc = 8;
    frame.data32[0] = 0xAAAAAAAA;
    frame.data32[1] = 0xAAAAAAAA;

    uint8_t mode = hex::parse(command.substr(0, 2));
    if (command.size() == 4 || command.size() == 5) {
        // Standard mode + PID (01 0C)
        uint8_t pid = hex::parse(command.substr(2, 2));
        frame.data[0] = 2;
        frame.data[1] = mode;
        frame.data[2] = pid;

        current_request.service = mode;
        current_request.pid = pid;

        if (command.size() == 5) {
            expected_frames = hex::parse(command.substr(4, 1));
        } else {
            expected_frames = 0;
        }

    } else if (command.size() == 6 || command.size() == 7) {
        // Mode + 16-bit PID (09 0200)
        uint16_t pid = hex::parse(command.substr(2, 4));
        frame.data[0] = 3; // 3 bytes of data
        frame.data[1] = mode;
        frame.data[2] = pid >> 8;
        frame.data[3] = pid & 0xFF;

        current_request.service = mode;
        current_request.pid = pid;

        if (command.size() == 7) {
            expected_frames = hex::parse(command.substr(6, 1));
        } else {
            expected_frames = 0;
        }
    } else {
        // Invalid format
        Log::error << "ELM327: Invalid PID request format: " << command << "\n";
        out << "?\r\n>";
        return;
    }

    const auto res = can::send_can(bus, frame);
    if (res < 0) {
        Log::error << "ELM327: Failed to send CAN frame: " << res << "\n";
        out << "ERROR\r\n>"; // TODO
        return;
    }
    out << "\r";

    current_request.request_start_time = pdTICKS_TO_MS(xTaskGetTickCount());

    is_waiting_for_response = true;
    processed_response = false;
}


void emulator::handle_can_frame(const can2040_msg& frame) {
    const auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);

    if (!params.monitor_mode) {
        if (!is_waiting_for_response) {
            return;
        }

        if (params.use_extended_frames) {
            if (id < 0x18DAF100 || id > 0x18DAF1FF) {
                return; // ignore
            }
        } else {
            if (id != params.obd_header + 0x08) {
                if (id < 0x7e8 || id > 0x7ef) {
                    return; // ignore
                }
            }
        }
    }

    if (xQueueSend(response_queue, &frame, 0) != pdTRUE) {
        Log::error << "ELM327: Failed to enqueue CAN frame\n";
    }
}

void emulator::reset(bool settings, bool timeout) {
    if (settings) {
        params = elm327::settings{
            .obd_header = obd2_11bit_broadcast,
            .timeout = 250,
            .line_feed = false,
            .echo = false,
            .white_spaces = true,
            .dlc = false,
            .monitor_mode = false,
            .memory = false,
            .print_headers = false,
            .use_extended_frames = false,
            .adaptive_timing = true,
        };
    }
    if (timeout) {
        params.timeout = 1000;
        params.adaptive_timing = true;
    }
    current_request = {0, 0, 0};
}

void emulator::update_timeout(uint64_t now,
                              uint64_t response_time,
                              uint8_t service,
                              uint32_t pid) {}

void emulator::check_for_timeout() {
    const auto now = pdTICKS_TO_MS(xTaskGetTickCount());
    const auto diff = now - current_request.request_start_time;
    if (diff > params.timeout) {
        if (!processed_response) {
            Log::debug << "ELM327: Timeout after " << params.timeout
                       << "ms waiting for response\n";
            update_timeout(now, diff, current_request.service, current_request.pid);
        }
        is_waiting_for_response = false;
        expected_frames = 0;
        received_frames = 0;
        if (params.line_feed) {
            out << "\n>";
        } else {
            out << "\r>";
        }
        out.flush();
    }
}


void emulator::process_input_byte(uint8_t byte, std::vector<char>& buff) {
    if (params.echo) {
        out << byte;
    }

    if (byte == '\r' || byte == '\n') {
        std::string_view cmd(buff.data(), buff.size());
        handle(cmd);
        buff.clear();
    } else if (byte != 0) {
        if (byte == 0x7F || byte == '\b') {
            if (!buff.empty()) {
                buff.pop_back();
            }
        } else if (byte > ' ' && byte < 0x7F) {
            buff.push_back(std::toupper(byte));
        }
    }
}

void emulator::process_can_frame(const can2040_msg& frame) {
    auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);


    if (!params.monitor_mode) {
        const auto now = pdTICKS_TO_MS(xTaskGetTickCount());

        processed_response = true;
        received_frames++;

        if (params.adaptive_timing) {
            const auto diff = now - current_request.request_start_time;
            update_timeout(now, diff, current_request.service, current_request.pid);
        }
        // current_request.request_start_time = now;

        if (expected_frames > 0 && received_frames >= expected_frames) {
            if (params.line_feed) {
                out << "\n>";
            } else {
                out << "\r>";
            }
            is_waiting_for_response = false;
            expected_frames = 0;
            received_frames = 0;
        }
    }
    std::string outBuff{};
    format_frame_output(frame, id, outBuff);
    out << outBuff;
    out.flush();
}

void emulator::format_frame_output(const can2040_msg& frame,
                                   uint32_t id,
                                   std::string& outBuff) const {
    // Max size: 8 bytes for header + 1 for space + 2 for DLC + 1 for space +
    // (frame.data[0] * 3) for data bytes and spaces + 1 for \r
    outBuff.clear();
    outBuff.reserve(11 + (frame.data[0] * 3));

    static constexpr char HEX_CHARS[] = "0123456789ABCDEF";

    if (params.print_headers) {
        if (params.use_extended_frames) {
            char header[7];
            header[0] = HEX_CHARS[(id >> 20) & 0xF];
            header[1] = HEX_CHARS[(id >> 16) & 0xF];
            header[2] = HEX_CHARS[(id >> 12) & 0xF];
            header[3] = HEX_CHARS[(id >> 8) & 0xF];
            header[4] = HEX_CHARS[(id >> 4) & 0xF];
            header[5] = HEX_CHARS[id & 0xF];
            header[6] = '\0';
            outBuff.append(header);
            if (params.white_spaces) {
                outBuff.push_back(' ');
            }
        } else {
            char header[4];
            header[0] = HEX_CHARS[(id >> 8) & 0xF];
            header[1] = HEX_CHARS[(id >> 4) & 0xF];
            header[2] = HEX_CHARS[id & 0xF];
            header[3] = '\0';
            outBuff.append(header);
            if (params.white_spaces) {
                outBuff.push_back(' ');
            }
        }
    }

    if (params.dlc) {
        outBuff.push_back(HEX_CHARS[frame.data[0] & 0xF]);
        if (params.white_spaces) {
            outBuff.push_back(' ');
        }
    }


    for (size_t i = 0; i < frame.data[0] && i < frame.dlc; i++) {
        uint8_t byte = frame.data[1 + i];
        outBuff.push_back(HEX_CHARS[(byte >> 4) & 0xF]);
        outBuff.push_back(HEX_CHARS[byte & 0xF]);

        if (params.white_spaces && i < frame.data[0] - 1 && i < frame.dlc - 1) {
            outBuff.push_back(' ');
        }
    }

    outBuff.push_back('\r');
}

bool emulator::is_valid_hex(std::string_view str) {
    return std::all_of(str.begin(), str.end(), [](char byte) {
        return (byte >= '0' && byte <= '9') || (byte >= 'A' && byte <= 'F') ||
               (byte >= 'a' && byte <= 'f');
    });
}


void emulator::emulator_task(void* params) {
    auto* self = static_cast<emulator*>(params);

    can2040_msg frame;
    uint8_t byte = 0;
    std::vector<char> buff;
    buff.reserve(64);
    while (self->running) {
        while (xQueueReceive(self->response_queue,
                             &frame,
                             self->is_waiting_for_response ? 2 : 0) == pdPASS) {
            self->process_can_frame(frame);
        }

        while (xQueueReceive(self->cmd_rx_queue, &byte, 0) == pdPASS) {
            self->process_input_byte(byte, buff);
        }
        if (self->is_waiting_for_response) {
            self->check_for_timeout();
        }
    }
}

} // namespace piccante::elm327