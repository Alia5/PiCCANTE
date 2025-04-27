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
#include <cmath>
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
    BaseType_t res = xTaskCreate(
        emulator_task, "ELM327 Emu", configMINIMAL_STACK_SIZE, this, 20, &task_handle);

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
    uint16_t pid = 0;
    if (command.size() == 4 || command.size() == 5) {
        // Standard mode + PID (01 0C)
        pid = hex::parse(command.substr(2, 2));
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
        pid = hex::parse(command.substr(2, 4));
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
    current_request.request_start_time = pdTICKS_TO_MS(xTaskGetTickCount());

    is_waiting_for_response = true;
    processed_response = false;
    const auto res = can::send_can(bus, frame);
    if (res < 0) {
        Log::error << "ELM327: Failed to send CAN frame: " << res << "\n";
        out << "ERROR\r\n>"; // TODO
        is_waiting_for_response = false;
        return;
    }

    // Log::debug << "ELM327: PID Request: " << command << "\n";
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
            if (id < 0x7e8 || id > 0x7ef) {
                return; // ignore
            }
        }
    }

    if (!xSemaphoreTake(mtx, pdMS_TO_TICKS(10))) {
        Log::error << "ELM327: Failed to take mutex\n";
        return;
    }

    process_can_frame(frame);
    xSemaphoreGive(mtx);
}

void emulator::reset(bool settings, bool timeout) {
    if (settings) {
        params = elm327::settings{
            .obd_header = obd2_11bit_broadcast,
            .timeout = 250,
            .line_feed = false,
            .echo = true,
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
        params.timeout = 500;
        params.adaptive_timing = true;
        average_response_time = 500;
    }
    current_request = {0, 0, 0};
}

void emulator::update_timeout(uint64_t now,
                              uint64_t response_time,
                              uint8_t service,
                              uint32_t pid) {
    static uint64_t avg_response_time = 1000;
    avg_response_time = (avg_response_time * 7 + response_time * 3) / 10;

    uint64_t new_timeout = avg_response_time * 4;

    new_timeout = std::max(new_timeout, min_timeout_ms);
    new_timeout = std::min(new_timeout, max_timeout_ms);

    params.timeout = new_timeout;

    // Log::debug << "ELM327: Adjusted timeout to " << params.timeout
    //            << "ms (avg response: " << avg_response_time << "ms)\n";
}


void emulator::check_for_timeout() {
    if (!is_waiting_for_response || params.monitor_mode) {
        return;
    }
    const auto now = pdTICKS_TO_MS(xTaskGetTickCount());

    // If we've received responses, use a short grace period to wait for more
    if (received_frames > 0) {
        // Short grace period (150ms) after last response to allow for other ECUs
        if (now - last_response_time > 250) {
            Log::debug << "Grace Period expired, stopping waiting for response\n";
            Log::debug << "Requeststart " << current_request.request_start_time
                       << ", last_response_time: " << last_response_time
                       << ", now: " << now
                       << ", Received Frames: " << uint32_t(received_frames)
                       << ", Expected Frames: " << uint32_t(expected_frames) << "\n";
            Log::debug << "Service: " << fmt::sprintf("%02X", current_request.service)
                       << ", PID: " << fmt::sprintf("%04X", current_request.pid) << "\n";
            Log::debug << "Timeout: " << params.timeout << "\n";
            Log::debug << "Last command: " << last_command << "\n\n";

            // We've received responses and waited a bit for more - done!
            is_waiting_for_response = false;
            expected_frames = 0;
            received_frames = 0;

            if (params.line_feed) {
                out << "\n>";
            } else {
                out << "\r>";
            }
            out.flush();
            return;
        }
    } else {
        // No responses yet, check main timeout
        const auto diff = now - current_request.request_start_time;
        if (diff > params.timeout) {
            if (!processed_response) {
                Log::debug << "ELM327: Timeout after " << params.timeout
                           << "ms waiting for response\n";
                Log::debug << "Service: " << fmt::sprintf("%02X", current_request.service)
                           << ", PID: " << fmt::sprintf("%04X", current_request.pid)
                           << "\n";
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


    format_frame_output(frame, id, outBuff);
    out << outBuff;
    // Log::debug << "ELM327: CAN Frame: " << outBuff << "\n";

    // Send flow control for multi-frame responses (ISO-TP)
    if (frame.data[0] == 0x10 && !params.monitor_mode) {
        // First frame of multi-frame message - need to send flow control
        can2040_msg fc_frame{};

        if (params.use_extended_frames) {
            uint32_t source_addr = id & 0xFF;
            uint32_t target_addr = (id & ~0xFF) | 0xF0;
            fc_frame.id = target_addr;
            fc_frame.id |= CAN2040_ID_EFF;
        } else {
            fc_frame.id = (id & ~0x07) | ((id & 0x07) ^ 0x08);
        }

        // Set flow control data
        fc_frame.dlc = 8;
        fc_frame.data[0] = 0x30; // Flow control
        fc_frame.data[1] = 0x00; // Block size (0 = no limit)
        fc_frame.data[2] = 0x00; // Separation time (0 = as fast as possible)

        // Fill remaining bytes with padding
        for (int i = 3; i < 8; i++) {
            fc_frame.data[i] = 0x00; // TODO: padding? Clone uses 0x00
        }

        // Send the flow control frame
        can::send_can(bus, fc_frame);
        Log::debug << "ELM327: Sent flow control frame to "
                   << fmt::sprintf("%08X", fc_frame.id) << "\n";
    }

    if (!params.monitor_mode) {
        const auto now = pdTICKS_TO_MS(xTaskGetTickCount());

        last_response_time = now;
        processed_response = true;
        received_frames++;

        if (params.adaptive_timing && received_frames == 1) {
            const auto diff = now - current_request.request_start_time;
            update_timeout(now, diff, current_request.service, current_request.pid);
        }

        if (expected_frames > 0 && received_frames >= expected_frames) {
            if (params.line_feed) {
                out << "\n>";
            } else {
                out << "\r>";
            }
            is_waiting_for_response = false;
            expected_frames = 0;
            received_frames = 0;
            // Log::debug << "ELM327: Received all expected frames\n";
        }
    }
    out.flush();
}

void emulator::format_frame_output(const can2040_msg& frame,
                                   uint32_t id,
                                   std::string& outBuff) const {
    outBuff.clear();
    const auto is_multi_frame = (frame.data[0] >= 0x10);
    if (is_multi_frame) {
        // first byte is "0x10" for every first multi
        // next frame first byte is 21 for second frame 22 for third and so on.
        // second byte is number of data bytes (frist frame only)
        // third is service+0x40
        // fourth is PID
        // rest are data bytes

        if (!params.print_headers) {
            if (frame.data[0] == 0x10) { // first frame?
                // print the second nibble of the first
                // byte and the second byte
                outBuff.push_back(HEX_CHARS[(frame.data[0]) & 0xF]);
                outBuff.push_back(HEX_CHARS[(frame.data[1] >> 4) & 0xF]);
                outBuff.push_back(HEX_CHARS[(frame.data[1]) & 0xF]);
                outBuff.push_back('\r');
            }
            // print sequence number
            outBuff.push_back(HEX_CHARS[frame.data[0] & 0xF]);
            outBuff.push_back(':');
            outBuff.push_back(' ');
        }

        // continue with the rest of the frame
    }

    if (params.print_headers) {
        if (params.use_extended_frames) {
            for (int i = 7; i >= 0; i--) {
                outBuff.push_back(HEX_CHARS[(id >> (i * 4)) & 0xF]);
                if (params.white_spaces && i % 2 == 0) {
                    outBuff.push_back(' ');
                }
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
        if (!is_multi_frame) {
            // When headers are enabled, always output the DLC as a 2-digit hex value
            // China ELM v1.5 (lol) does seem to do this... ¯\_(ツ)_/¯
            outBuff.push_back(HEX_CHARS[(frame.dlc >> 4) & 0xF]);
            outBuff.push_back(HEX_CHARS[frame.dlc & 0xF]);

            if (params.white_spaces) {
                outBuff.push_back(' ');
            }
        }
    }


    if (!params.print_headers && !is_multi_frame && params.dlc) {
        outBuff.push_back(HEX_CHARS[frame.data[0] & 0xF]);

        if (params.white_spaces) {
            outBuff.push_back(' ');
        }
    }

    const auto from = is_multi_frame                                                 //
                          ? frame.data[0] == 0x10 ? params.print_headers ? 0 : 2 : 0 //
                          : 1;
    const auto to = is_multi_frame  //
                        ? frame.dlc //
                        : std::min(frame.data[0] + 1, (int)frame.dlc);

    for (size_t i = from; i < to; i++) {
        const uint8_t byte = frame.data[i];
        outBuff.push_back(HEX_CHARS[(byte >> 4) & 0xF]);
        outBuff.push_back(HEX_CHARS[byte & 0xF]);

        if (params.white_spaces && i < to - 1) {
            outBuff.push_back(' ');
        }
    }

    outBuff.push_back('\r');
}


void emulator::emulator_task(void* params) {
    auto* self = static_cast<emulator*>(params);

    uint8_t byte = 0;
    std::vector<char> buff;
    buff.reserve(64);
    while (self->running) {
        if (self->is_waiting_for_response) {
            if (xSemaphoreTake(self->mtx, 0) == pdTRUE) {
                self->check_for_timeout();
                xSemaphoreGive(self->mtx);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            while (xQueueReceive(self->cmd_rx_queue, &byte, pdMS_TO_TICKS(10)) ==
                   pdPASS) {
                self->process_input_byte(byte, buff);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

} // namespace piccante::elm327