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
#include <can2040.h>
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

    // TODO: check if only on \n or also on \r
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
    frame.data32[0] = 0xCCCCCCCC;
    frame.data32[1] = 0xCCCCCCCC;

    uint8_t mode = hex::parse(command.substr(0, 2));
    if (command.size() == 4 || command.size() == 5) {
        // Standard mode + PID (01 0C)
        uint8_t pid = hex::parse(command.substr(2, 2));
        frame.data[0] = 2;
        frame.data[1] = mode;
        frame.data[2] = pid;

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

    last_can_event_time = pdTICKS_TO_MS(xTaskGetTickCount());

    // Apply increased timeout for multi-frame responses
    if (expected_frames > 0) {
        params.timeout = std::min(average_response_time_ms * TIMEOUT_SAFETY_FACTOR *
                                      MULTI_FRAME_TIMEOUT_MULTIPLIER,
                                  max_timeout_ms);
    }

    is_waiting_for_response = true;
    processed_response = false;
}


void emulator::handle_can_frame(const can2040_msg& frame) {
    auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);

    if (!params.monitor_mode) {
        if (!is_waiting_for_response) {
            return;
        }
        if (params.use_extended_frames) {
            // is an answer to us?
            // If the vehicle responds, you will see responses with CAN IDs 0x18DAF100
            // to 0x18DAF1FF (typically 18DAF110 and 18DAF11E).
            if (id < 0x18DAF100 || id > 0x18DAF1FF) {
                return; // ignore
            }
            // Remove the leading "18" prefix for extended addressing
            id &= 0x00FFFFFF;
        } else {
            if (id != params.obd_header + 0x08) {
                // is an answer to us?
                if (id < 0x7e8 || id > 0x7ef) {
                    return; // ignore
                }
            }
        }
    }


    std::string outBuff{};
    if (params.print_headers) {
        if (params.use_extended_frames) {
            outBuff += fmt::sprintf("%06X ", id); // Output only 6 hex characters
        } else {
            outBuff += fmt::sprintf("%03X ", id);
        }
    }

    if (params.white_spaces) {
        outBuff += " ";
    }

    if (params.dlc) {
        outBuff += fmt::sprintf("%u ", frame.dlc);
    }

    if (params.white_spaces) {
        outBuff += " ";
    }


    for (size_t i = 0; i < frame.data[0] && i < frame.dlc; i++) {
        outBuff += fmt::sprintf("%02X", frame.data[1 + i]);
        if (params.white_spaces && i < frame.dlc - 1) {
            outBuff += " ";
        }
    }
    out << outBuff;
    out << "\r";

    if (!params.monitor_mode) {
        const auto now = pdTICKS_TO_MS(xTaskGetTickCount());
        if (params.adaptive_timing) {
            update_timeout(now);
        }
        last_can_event_time = now;
        processed_response = true;
        received_frames++;
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

    out.flush();
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
        params.timeout = 250;
        params.adaptive_timing = true;
        average_response_time_ms = 150;
    }
}

void emulator::update_timeout(uint64_t now) {
    const auto diff = now - last_can_event_time;

    if (average_response_time_ms < diff) {
        // Response is slower than expected - increase quickly
        const uint8_t total_weight = SLOW_RESPONSE_WEIGHT + SLOW_RESPONSE_HISTORY_WEIGHT;
        average_response_time_ms =
            ((diff * SLOW_RESPONSE_WEIGHT) +
             (SLOW_RESPONSE_HISTORY_WEIGHT * average_response_time_ms)) /
            total_weight;
    } else {
        // Response is faster than expected - decrease very slowly
        const uint8_t total_weight = FAST_RESPONSE_WEIGHT + FAST_RESPONSE_HISTORY_WEIGHT;
        average_response_time_ms =
            ((diff * FAST_RESPONSE_WEIGHT) +
             (FAST_RESPONSE_HISTORY_WEIGHT * average_response_time_ms)) /
            total_weight;
    }

    params.timeout = std::min(
        std::max((average_response_time_ms * TIMEOUT_SAFETY_FACTOR), min_timeout_ms),
        max_timeout_ms);
}

void emulator::check_for_timeout() {
    const auto now = pdTICKS_TO_MS(xTaskGetTickCount());
    if (last_can_event_time + params.timeout < now) {
        if (!processed_response) {
            Log::debug << "ELM327: Timeout after " << params.timeout
                       << "ms waiting for response\n";
            update_timeout(now);
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

bool emulator::is_valid_hex(std::string_view str) {
    return std::all_of(str.begin(), str.end(), [](char byte) {
        return (byte >= '0' && byte <= '9') || (byte >= 'A' && byte <= 'F') ||
               (byte >= 'a' && byte <= 'f');
    });
}


void emulator::emulator_task(void* params) {
    auto* self = static_cast<emulator*>(params);

    uint8_t byte = 0;
    std::vector<char> buff;
    buff.reserve(64);
    while (self->running) {
        auto received = false;
        if (xQueueReceive(self->cmd_rx_queue, &byte, 0) == pdTRUE) {
            if (self->params.echo) {
                self->out << byte;
            }
            if (byte == '\r' || byte == '\n') {
                std::string_view cmd(buff.data(), buff.size());
                self->handle(cmd);
                buff.clear();
            } else {
                if (byte != 0) {
                    if (byte == 0x7F || byte == '\b') {
                        if (!buff.empty()) {
                            buff.pop_back();
                        }
                    }
                    // only push back valid ascii and strip spaces
                    if (byte > ' ' && byte < 0x7F) { // 0x7F = del, 0x7E = ~
                        buff.push_back(std::toupper(byte));
                    }
                }
            }
            received = true;
        }
        if (self->is_waiting_for_response) {
            self->check_for_timeout();
            vTaskDelay(1);
        }
        if (!received && !self->is_waiting_for_response) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

} // namespace piccante::elm327