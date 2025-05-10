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
#include "CanBus/frame.hpp"
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
    running = false;
    while (task_handle) {
        if (eTaskGetState(task_handle) == eDeleted) {
            task_handle = nullptr;
            break;
        }
        xTaskNotifyGive(task_handle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool emulator::is_running() const { return running; }

int emulator::handle(std::string_view command) {
    if (command.empty()) {
        return 0;
    }

    if (command[0] == '\r' || command[0] == '\n') {
        return handle(last_command);
    }
    last_command = command;

    if (is_valid_hex(command)) {
        // assume every only hex command is a valid pid request...
        return handle_pid_request(command);
    }
    Log::debug << "ELM327: command: " << command << "\n";
    // TODO: check if command is valid.
    handle_command(command);
    return 0;
}
void emulator::handle_command(std::string_view command) {
    if (at_h.is_at_command(command)) {
        at_h.handle(command);
        return;
    }
    if (st_h.is_st_command(command)) {
        st_h.handle(command);
        return;
    }
    Log::error << "ELM327: Invalid command: " << command << "\n";
    out << "?\r\n>";
    out.flush();
}

int emulator::handle_pid_request(std::string_view command) {
    piccante::can::frame frame{};

    frame.id = params.obd_header;
    if (params.use_extended_frames) {
        frame.extended = true;
    }

    frame.dlc = 8;
    frame.data32[0] = 0xAAAAAAAA;
    frame.data32[1] = 0xAAAAAAAA;

    uint8_t mode = hex::parse(command.substr(0, 2));
    uint16_t pid = 0;

    constexpr uint8_t max_command_size =
        6 * 2 +
        1; // 6 bytes of data (8 byte can frame - mode - OBD2-dlc) + 1 for expected frames
    if (command.size() < 2 || command.size() > max_command_size) {
        Log::error << "ELM327: Invalid PID request format: " << command << "\n";
        out << "?\r\n>";
        out.flush();
        return 0;
    }
    current_request.return_after_n_frames =
        command.size() % 2 == 0 ? 0 : hex::parse(command.substr(command.size() - 1, 1));
    const auto len =
        (command.size() - 2 - (current_request.return_after_n_frames > 0 ? 1 : 0)) / 2;
    pid = hex::parse(command.substr(2, 2 * len));

    current_request.service = mode;
    current_request.pid = pid;

    frame.data[0] = len + 1;
    frame.data[1] = mode;

    for (uint8_t i = 0; i < len; i++) {
        frame.data[i + 2] = pid >> ((len - 1 - i) * 8);
    }
    current_request.request_start_time = pdTICKS_TO_MS(xTaskGetTickCount());

    is_waiting_for_response = true;
    current_request.processed_response = false;
    const auto res = can::send_can(bus, frame);
    if (res < 0) {
        is_waiting_for_response = false;
        Log::error << "ELM327: Failed to send CAN frame: " << res << "\n";
        out << (params.line_feed ? "?\r\n>" : "?\r\r>");
        out.flush();
        return 0;
    }
    return params.timeout;
}


void emulator::handle_can_frame(const can::frame& frame) {
    if (!params.monitor_mode) {
        if (!is_waiting_for_response) {
            return;
        }

        if (params.use_extended_frames) {
            if (frame.id < 0x18DAF100 || frame.id > 0x18DAF1FF) {
                return; // ignore
            }
        } else {
            if (frame.id < 0x7e8 || frame.id > 0x7ef) {
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
    }
    current_request.return_after_n_frames = 0;
    current_request.received_frames = 0;
    current_request.processed_response = false;
    current_request.request_start_time = 0;
    current_request.last_response_time = 0;
}

void emulator::update_timeout(uint64_t now,
                              uint64_t response_time,
                              uint8_t service,
                              uint32_t pid) {
    // TODO:
}


int emulator::check_for_timeout() {
    if (!is_waiting_for_response || params.monitor_mode) {
        return 1;
    }
    const auto now = pdTICKS_TO_MS(xTaskGetTickCount());

    const auto diff = now - current_request.last_response_time;
    if (diff > params.timeout) {
        if (!current_request.processed_response) {
            Log::debug << "ELM327: Timeout after " << params.timeout
                       << "ms waiting for response\n";
            Log::debug << "Service: " << fmt::sprintf("%02X", current_request.service)
                       << ", PID: " << fmt::sprintf("%04X", current_request.pid) << "\n";
            update_timeout(now, diff, current_request.service, current_request.pid);
        }
        is_waiting_for_response = false;
        current_request.return_after_n_frames = 0;
        current_request.received_frames = 0;
        current_request.processed_response = false;
        current_request.request_start_time = 0;
        current_request.last_response_time = 0;

        if (params.line_feed) {
            out << "\n>";
        } else {
            out << "\r>";
        }
        out.flush();
        return 0;
    }
    return diff;
}


int emulator::process_input_byte(uint8_t byte, std::vector<char>& buff) {
    if (params.echo) {
        out << byte;
    }

    if (byte == '\r' || byte == '\n') {
        std::string_view cmd(buff.data(), buff.size());
        const auto res = handle(cmd);
        buff.clear();
        return res;
    } else if (byte != 0) {
        if (byte == 0x7F || byte == '\b') {
            if (!buff.empty()) {
                buff.pop_back();
            }
        } else if (byte > ' ' && byte < 0x7F) {
            buff.push_back(std::toupper(byte));
        }
    }
    return 0;
}

void emulator::process_can_frame(const can::frame& frame) {
    auto id = frame.id;

    const auto now = pdTICKS_TO_MS(xTaskGetTickCount());
    current_request.last_response_time = now;
    current_request.received_frames++;


    format_frame_output(frame, id, outBuff);
    out << outBuff;
    if (params.monitor_mode) {
        out.flush();
        xTaskNotifyGive(task_handle);
        return;
    }

    // Send flow control for multi-frame responses (ISO-TP)
    if (frame.data[0] == iso_tp::FirstFrame) {
        // First frame of multi-frame message - need to send flow control
        piccante::can::frame fc_frame{};

        uint32_t fc_target_id = id;
        if (params.use_extended_frames) {
            uint32_t byte0 = id & 0xFF;
            uint32_t byte1 = (id >> 8) & 0xFF;
            fc_target_id = (id & 0xFFFF0000) | (byte0 << 8) | byte1;
            fc_frame.id = fc_target_id;
            fc_frame.extended = true;
        } else {
            fc_target_id = id - 0x08;
            fc_frame.id = fc_target_id;
        }

        const auto len = frame.data[1];
        const auto left_len = len - 6;
        const auto left_frames = (left_len / 7) + (left_len % 7 > 0 ? 1 : 0);

        if (params.adaptive_timing) {
            if (current_request.received_frames == 1) {
                current_request.return_after_n_frames = left_frames + 1; // + this frame
            } else {
                // could be response from another ecu, add up.
                current_request.return_after_n_frames += left_frames + 1; // + this frame
            }
        }


        // Set flow control data
        fc_frame.dlc = 8;
        fc_frame.data[0] = iso_tp::FlowControl;
        fc_frame.data[1] = 0x00; // Block size (0 = no limit)
        fc_frame.data[2] = 0x00; // Separation time (0 = as fast as possible)

        // Fill remaining bytes with padding
        for (int i = 3; i < 8; i++) {
            fc_frame.data[i] = 0x00; // TODO: padding? Clone uses 0x00
        }

        // Send the flow control frame
        const auto send_fc_res = can::send_can(bus, fc_frame);
        taskYIELD();
        if (send_fc_res < 0) {
            Log::error << "ELM327: Failed to send flow control frame: " << send_fc_res
                       << "\n";
        } else {
            Log::debug << "ELM327: Sent flow control frame to "
                       << fmt::sprintf("%08X", fc_target_id) << "\n";
        }
    }

    if (current_request.return_after_n_frames > 0) {
        if (current_request.received_frames >= current_request.return_after_n_frames) {
            current_request.processed_response = true;
        }
    } else {
        current_request.processed_response = true;
    }

    if (params.adaptive_timing && current_request.received_frames == 1) {
        const auto diff = now - current_request.request_start_time;
        update_timeout(now, diff, current_request.service, current_request.pid);
    }

    if (current_request.return_after_n_frames > 0 &&
        current_request.received_frames >= current_request.return_after_n_frames) {
        Log::debug << "ELM327: Received " << int(current_request.received_frames)
                   << " of " << int(current_request.return_after_n_frames)
                   << " frames, returning prompt\n";
        if (params.line_feed) {
            out << "\n>";
        } else {
            out << "\r>";
        }
        out.flush();
        is_waiting_for_response = false;
        current_request.return_after_n_frames = 0;
        current_request.received_frames = 0;
        current_request.processed_response = false;
        current_request.request_start_time = 0;
        current_request.last_response_time = 0;
        xTaskNotifyGive(task_handle);
        // Log::debug << "ELM327: Received all expected frames\n";
    }
}

void emulator::format_frame_output(const can::frame& frame,
                                   uint32_t id,
                                   std::string& outBuff) const {
    outBuff.clear();
    const auto is_multi_frame =
        (frame.data[0] >=
         iso_tp::FirstFrame); // works, because otherwise first frame would be DLC < 8
    if (is_multi_frame) {
        if (!params.print_headers) {
            if (frame.data[0] == iso_tp::FirstFrame) { // first frame?
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
    int wait_time = 0;
    while (self->running) {
        if (self->is_waiting_for_response) {
            if (xSemaphoreTake(self->mtx, 0) == pdTRUE) {
                wait_time = self->check_for_timeout();
                xSemaphoreGive(self->mtx);
            }
            if (wait_time > 0) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_time));
            }
        } else {
            while (self->running &&
                   xQueueReceive(self->cmd_rx_queue, &byte, pdMS_TO_TICKS(10)) ==
                       pdPASS) {
                wait_time = self->process_input_byte(byte, buff);
                if (wait_time > 0) {
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_time));
                    break;
                }
            }
        }
    }
    vTaskDelete(nullptr);
    self->task_handle = nullptr;
}

} // namespace piccante::elm327