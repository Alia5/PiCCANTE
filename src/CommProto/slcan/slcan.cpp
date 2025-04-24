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

/*
 * SLCAN/Can232 Protocol implementation based on
 * https://www.canusb.com/files/canusb_manual.pdf
 * V2 (extended_mode) additions are found in the linked file;
 * I don't know if this is actually correct or of Collin80 is the original Author
 * ¯\_(ツ)_/¯ https://github.com/collin80/ESP32RET/blob/master/SLCAN.cpp
 */

#include "slcan.hpp"
#include <can2040.h>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <algorithm>
#include <charconv>
#include <projdefs.h>
#include <ranges>
#include <system_error>
#include <functional>
#include "outstream/stream.hpp"
#include <utility>
#include "../CanBus/CanBus.hpp"
#include "../util/hexstr.hpp"
#include "fmt.hpp"
#include "Logger/Logger.hpp"
#include "FreeRTOS.h"
#include <class/cdc/cdc_device.h>

namespace piccante::slcan {

constexpr uint8_t NUM_BUSES = 1; // For now a single bus per slcan interface


TaskHandle_t& handler::create_task(UBaseType_t priority) {
    xTaskCreate(handler::task_dispatcher, fmt::sprintf("SLCAN%d", bus).c_str(),
                configMINIMAL_STACK_SIZE, this, priority, &task_handle);
    return task_handle;
}
void handler::handle_short_cmd(char cmd) {
    Log::debug << "SLCAN Short Command:" << cmd << "\n";

    switch (cmd) {
        case SHORT_CMD::OPEN: // SLCAN open canbus port
            can::enable(bus, can::get_bitrate(bus));
            can::set_listenonly(bus, false);
            host_out << '\r'; // ok
            host_out.flush();
            break;
        case SHORT_CMD::CLOSE: // SLCAN close canbus port (First one)
            can::disable(bus);
            host_out << '\r'; // ok
            host_out.flush();
            break;
        case SHORT_CMD::OPEN_LISTEN_ONLY: // SLCAN open canbus port in listen only
            can::enable(bus, can::get_bitrate(bus));
            can::set_listenonly(bus, true);
            host_out << '\r'; // ok
            host_out.flush();
            break;
        case SHORT_CMD::POLL_ONE: // SLCAN - poll for one waiting frame. Or, just CR
            poll_counter++;
            break;
        case SHORT_CMD::POLL_ALL: // SLCAN - poll for all waiting frames - CR if no
            poll_counter = can::get_can_rx_buffered_frames(0);
            if (poll_counter == 0) {
                host_out << '\r'; // ok
                host_out.flush();
            }
            break;
        case SHORT_CMD::READ_STATUS_BITS: // SLCAN - read status bits
            host_out << "F00"; // bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error
            host_out << '\r';  // ok
            host_out.flush();
            break;
        case SHORT_CMD::GET_VERSION: // SLCAN - get version number
            host_out << ("V1013\n"); // ok
            host_out.flush();
            break;
        case SHORT_CMD::GET_SERIAL:     // SLCAN - get serial number
            host_out << ("PiCCANTE\n"); // ok
            host_out.flush();
            // SysSettings.SLCANMode = true;
            break;
        case SHORT_CMD::SET_EXTENDED_MODE:
            extended_mode = !extended_mode;
            host_out << (extended_mode ? "V2\n" : "SLCAN\n"); // ok
            host_out.flush();

            break;
        case SHORT_CMD::LIST_SUPPORTED_BUSSES: // SLCAN V2 - Output list of
            if (extended_mode) {
                for (int i = 0; std::cmp_less(i, NUM_BUSES); i++) {
                    printBusName();
                    host_out << '\r'; // ok
                    host_out.flush();
                }
            }
            break;
        case SHORT_CMD::FIRMWARE_UPGRADE:
            // Not supported
            host_out << '\a'; // err
            host_out.flush();
            break;
        default:
            Log::error << "Unknown command:" << cmd << "\n";
            break;
    }
}

void handler::handle_long_cmd(const std::string_view& cmd) {
    can2040_msg out_frame{};


    Log::debug << "SLCAN Long Command:" << cmd << "\n";

    switch (cmd[0]) {
        {
            case LONG_CMD::TX_STANDARD_FRAME:
                out_frame.id = piccante::hex::parse(cmd.substr(1, 3));
                out_frame.dlc = std::clamp(cmd.data()[4] - '0', 0, 8);
                for (unsigned int i = 0; i < out_frame.dlc; i++) {
                    out_frame.data[i] = piccante::hex::parse(cmd.substr(5 + (i * 2), 2));
                }
                can::send_can(bus, out_frame);
                if (auto_poll) {
                    host_out << ("z");
                }
                break;
            case LONG_CMD::TX_EXTENDED_FRAME:
                out_frame.id = piccante::hex::parse(cmd.substr(1, 8));
                out_frame.dlc = std::clamp(cmd.data()[9] - '0', 0, 8);
                for (size_t data = 0; data < out_frame.dlc; data++) {
                    out_frame.data[data] =
                        piccante::hex::parse(cmd.substr(10 + (2 * data), 2));
                }
                can::send_can(bus, out_frame);
                if (auto_poll) {
                    host_out << ("Z");
                }
                break;
            case LONG_CMD::SET_SPEED_OR_PACKET:
                if (extended_mode) {
                    // Send packet to specified BUS

                    auto parts = cmd | std::views::split(' ');

                    auto it = std::ranges::begin(parts);
                    if (it != std::ranges::end(parts)) {
                        ++it;
                    }
                    if (it != std::ranges::end(parts)) {
                        std::string_view bus_name((*it).begin(), (*it).end());
                        if (bus_name != "CAN0") {
                            host_out << ("Unknown bus\n");
                            return;
                        }
                        // ok
                    }
                    ++it;

                    if (it == std::ranges::end(parts)) {
                        host_out << "Missing CAN ID\n";
                        return;
                    }
                    std::string_view id_str((*it).begin(), (*it).end());
                    uint32_t id = piccante::hex::parse(id_str);
                    ++it;

                    if (it == std::ranges::end(parts)) {
                        host_out << "No data bytes provided\n";
                        return;
                    }

                    std::string_view data_hex((*it).begin(), (*it).end());
                    if (data_hex.empty()) {
                        host_out << "No data bytes provided\n";
                        return;
                    }

                    can2040_msg out_frame{};
                    out_frame.id = id;
                    uint8_t num_bytes =
                        std::min(static_cast<size_t>(8), data_hex.length() / 2);

                    // Parse each pair of hex chars as one byte
                    for (uint8_t i = 0; i < num_bytes; i++) {
                        std::string_view byte_str = data_hex.substr(i * 2, 2);
                        out_frame.data[i] = piccante::hex::parse(byte_str);
                    }

                    out_frame.dlc = num_bytes;
                    can::send_can(bus, out_frame);

                    if (auto_poll) {
                        host_out << "z";
                    }
                } else {
                    uint8_t speedNum = std::clamp(
                        cmd[1] - '0', 0, static_cast<int>(bus_speeds.size() - 1));
                    can::set_bitrate(bus, bus_speeds[speedNum]);
                }
                break;
            case LONG_CMD::SET_CANBUS_BAUDRATE:
                // TODO;
                break;
            case LONG_CMD::SEND_RTR_FRAME:
                // deprecated maybe.
                break;
            case LONG_CMD::SET_RECEIVE_TRAFFIC:
                if (extended_mode) {
                    can::enable(bus, can::get_bitrate(bus));
                } else {
                    // V1 send rtr frame, see above...
                }
                break;
            case LONG_CMD::SET_AUTOPOLL:
                auto_poll = cmd[1] == '1';
                break;
            case LONG_CMD::SET_FILTER_MODE:
                // unsupported.  // TODO: impl. softwareFilter
                break;
            case LONG_CMD::SET_ACCEPTENCE_MASK:
                // unsupported.  // TODO: impl. softwareFilter
                break;
            case LONG_CMD::SET_FILTER_MASK:
                if (extended_mode) {
                    // TODO: impl. softwareFilter
                } else {
                    // V1 set acceptance mode
                }
                break;
            case LONG_CMD::HALT_RECEPTION:
                if (extended_mode) {
                    can::disable(bus);
                }
                break;
            case LONG_CMD::SET_UART_SPEED:
                // USB_CDC IGNORES BAUD
                break;
            case LONG_CMD::TOGGLE_TIMESTAMP:
                time_stamping = cmd[1] == '1' || cmd[1] == '2';
                break;
            case LONG_CMD::TOGGLE_TIMESTAMP_ALT:
                time_stamping = cmd[1] == '1' || cmd[1] == '2';
                break;
            case LONG_CMD::TOGGLE_AUTO_START:
                // TODO: Maybe?
                break;
            case LONG_CMD::CONFIGURE_BUS:
                if (extended_mode) {
                    auto parts = cmd | std::views::split(' ');

                    auto it = std::ranges::begin(parts);
                    if (it != std::ranges::end(parts)) {
                        ++it;
                    }
                    if (it != std::ranges::end(parts)) {
                        std::string_view bus_name((*it).begin(), (*it).end());
                        if (bus_name != "CAN0") {
                            host_out << ("Unknown bus\n");
                            return;
                        }
                        // ok
                    }
                    ++it;
                    if (it != std::ranges::end(parts)) {
                        int speed{};
                        const auto convres =
                            std::from_chars((*it).begin(), (*it).end(), speed);
                        if (convres.ec == std::errc() && speed > 0) {
                            can::set_bitrate(bus, speed);
                        } else {
                            host_out << ("Invalid speed\n");
                            return;
                        }
                    }
                }
                break;
            default:
                Log::error << "Unknown command: " << cmd << "\n";
                break;
        }
    }
    host_out << ('\r'); // ok
    host_out.flush();
}
constexpr int EXT_ID_MAX_SHIFT = 28; // 32 - 4 (7 shifts of 4 bits)
constexpr std::string_view hex_digits = "0123456789abcdef";

void handler::comm_can_frame(const can2040_msg& frame) {
    uint32_t const millis = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (!auto_poll) {
        if (poll_counter > 0) {
            poll_counter--;
        } else {
            return;
        }
    }


    const bool is_extended = frame.id & CAN2040_ID_EFF;
    const auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);

    if (extended_mode) {
        host_out << std::to_string(millis) << " - " << fmt::sprintf("%x", id);
        host_out << (frame.id & CAN2040_ID_EFF ? " X " : " S "); // Extended frame
        printBusName();
        for (int i = 0; std::cmp_less(i, frame.dlc); i++) {
            // TODO: Provide more elegant write
            host_out << (" ");
            host_out << (fmt::sprintf("%x", frame.data[i]));
        }
    } else {
        std::size_t pos = 0;
        can_out_buffer[pos++] = is_extended ? 'T' : 't';

        if (is_extended) {
            for (int i = EXT_ID_MAX_SHIFT; i >= 0; i -= 4) {
                can_out_buffer[pos++] = hex_digits[(id >> i) & 0xF];
            }
        } else {
            can_out_buffer[pos++] = hex_digits[(id >> 8) & 0xF];
            can_out_buffer[pos++] = hex_digits[(id >> 4) & 0xF];
            can_out_buffer[pos++] = hex_digits[id & 0xF];
        }

        can_out_buffer[pos++] = '0' + frame.dlc;

        for (uint32_t i = 0; i < frame.dlc; i++) {
            const auto byte = frame.data[i];
            can_out_buffer[pos++] = hex_digits[(byte >> 4) & 0xF];
            can_out_buffer[pos++] = hex_digits[byte & 0xF];
        }
        host_out.write(can_out_buffer.data(), pos);
        if (time_stamping) {
            host_out << (fmt::sprintf("%04x", millis)) << '\0';
        }
    }
    host_out << ('\r'); // ok
    host_out.flush();
}

void handler::handle_command(const std::string_view& cmd) {
    if (cmd.size() < 2) {
        handle_short_cmd(cmd[0]);
    } else if (cmd.size() >= 2) {
        handle_long_cmd(cmd);
    }
}


void handler::printBusName() const {
    host_out << ("CAN0\n"); // ok
    host_out.flush();
}

void handler::task_dispatcher(void* p) { // NOLINT
    auto* h = static_cast<handler*>(p);  // NOLINT
    h->task();
}

void handler::task() {
    std::vector<char> buffer;
    buffer.reserve(64);
    for (;;) {
        auto received = false;
        while (tud_cdc_n_available(read_itf) > 0) {
            received = true;
            const auto c = tud_cdc_n_read_char(read_itf);
            if (c == '\r' || c == '\n') {
                std::string_view cmd(buffer.data(), buffer.size());
                handle_command(cmd);
                buffer.clear();
            } else {
                buffer.push_back(c);
            }
        }
        if (received) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(piccanteIDLE_SLEEP_MS));
        }
    }
}


} // namespace piccante::slcan