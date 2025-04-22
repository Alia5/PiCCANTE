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
    // TODO: build can frame and send to bus

    can2040_msg frame{};

    // TODO: Ecu Header?!
    frame.id = params.obd_header;

    if (params.use_extended_frames) {
        frame.id |= CAN2040_ID_EFF;
    }

    frame.dlc = 8;
    frame.data32[0] = 0xAAAAAAAA;
    frame.data32[1] = 0xAAAAAAAA;

    uint8_t mode = hex::parse(command.substr(0, 2));
    if (command.size() == 4) {
        uint8_t pid = hex::parse(command.substr(2, 2));
        frame.data[0] = 2;
        frame.data[1] = mode;
        frame.data[2] = pid;
        Log::debug << "ELM327: Mode: " << static_cast<int>(mode)
                   << ", PID: " << static_cast<int>(pid) << "\n";
    } else {
        uint16_t pid = hex::parse(command.substr(2, 4));
        frame.data[0] = 2;
        frame.data[1] = mode;
        frame.data[2] = pid >> 8;
        frame.data[3] = pid & 0xFF;
        Log::debug << "ELM327: Mode: " << static_cast<int>(mode)
                   << ", PID: " << static_cast<int>(pid) << "\n";
    }

    last_send_time = pdTICKS_TO_MS(xTaskGetTickCount());

    const auto res = can::send_can(bus, frame);
    if (res < 0) {
        Log::error << "ELM327: Failed to send CAN frame: " << res << "\n";
        out << "ERROR\r\n>"; // TODO
        return;
    }
}


void emulator::handle_can_frame(const can2040_msg& frame) {
    const auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);

    if (!params.monitor_mode) {
        if (params.use_extended_frames) {
            // is an answer to us?
            // If the vehicle responds, you will see responses with CAN IDs 0x18DAF100
            // to 0x18DAF1FF (typically 18DAF110 and 18DAF11E).
            if (id < 0x18DAF100 || id > 0x18DAF1FF) {
                return; // ignore
            }
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
            outBuff += fmt::sprintf("%08X ", id);
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

    for (size_t i = 0; i < frame.dlc; i++) {
        outBuff += fmt::sprintf("%02X", frame.data[i]);
        if (params.white_spaces && i < frame.dlc - 1) {
            outBuff += " ";
        }
    }
    out << outBuff;
    out << "\r";

    if (timeout_timer != nullptr) {
        xTimerDelete(timeout_timer, 0);
        timeout_timer = nullptr;
    }

    timeout_timer = xTimerCreate("ELMTimeout",
                                 pdMS_TO_TICKS(params.timeout),
                                 pdFALSE,
                                 this, // Timer ID = this object
                                 [](TimerHandle_t xTimer) {
                                     emulator* elm = static_cast<emulator*>(
                                         pvTimerGetTimerID(xTimer));
                                     if (elm->params.line_feed) {
                                         elm->out << "\r\n>";
                                     } else {
                                         elm->out << "\r>";
                                     }
                                     elm->out.flush();
                                     elm->timeout_timer = nullptr;
                                 });

    if (timeout_timer != nullptr) {
        xTimerStart(timeout_timer, 0);
    }

    if (!params.monitor_mode && params.adaptive_timing) {
        const auto now = pdTICKS_TO_MS(xTaskGetTickCount());
        const auto diff = now - last_send_time;
        params.timeout = std::min(std::max(diff * 2, min_timeout_ms), max_timeout_ms);
    }

    out.flush();
}

bool emulator::is_valid_hex(std::string_view str) {
    return std::all_of(str.begin(), str.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    });
}


void emulator::emulator_task(void* params) {
    auto* self = static_cast<emulator*>(params);

    uint8_t byte = 0;
    std::vector<char> buff;
    buff.reserve(64);
    while (self->running) {
        while (xQueueReceive(self->cmd_rx_queue, &byte, pdTICKS_TO_MS(100)) == pdTRUE) {
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
                        buff.push_back(byte);
                    }
                }
            }
        }
    }
}

} // namespace piccante::elm327