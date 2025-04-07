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
#pragma once

#include <string_view>
#include <cstdint>
#include <functional>
#include "outstream/stream.hpp"
#include <map>

struct can2040_msg;

namespace piccante::lawicel {


enum SHORT_CMD {
    OPEN = 'O',
    CLOSE = 'C',
    OPEN_LISTEN_ONLY = 'L',
    POLL_ONE = 'P',
    POLL_ALL = 'A',
    READ_STATUS_BITS = 'F',
    GET_VERSION = 'V',
    GET_SERIAL = 'N',
    SET_EXTENDED_MODE = 'x',
    LIST_SUPPORTED_BUSSES = 'B',
    FIRMWARE_UPGRADE = 'X',
    //
};

enum LONG_CMD {
    TX_STANDARD_FRAME = 't',
    TX_EXTENDED_FRAME = 'T',
    SET_SPEED_OR_PACKET = 'S',
    SET_CANBUS_BAUDRATE = 's',
    SEND_RTR_FRAME = 'r',
    SET_RECEIVE_TRAFFIC = 'R',
    SET_AUTOPOLL = 'A',
    SET_FILTER_MODE = 'W',
    SET_ACCEPTENCE_MASK = 'm',
    SET_FILTER_MASK = 'M',
    HALT_RECEPTION = 'H',
    SET_UART_SPEED = 'U',
    TOGGLE_TIMESTAMP = 'Z',
    TOGGLE_AUTO_START = 'Q',
    CONFIGURE_BUS = 'C',
};

class handler {
        public:
    explicit handler(out::stream& out_stream, uint8_t bus_num = 0)
        : host_out(out_stream), bus(bus_num) {};
    virtual ~handler() = default;


    void handleShortCmd(char cmd);
    void handleLongCmd(const std::string_view& cmd);
    void handleCmd(const std::string_view& cmd);
    void handleCanFrame(const can2040_msg& frame);


        private:
    std::reference_wrapper<out::stream> host_out;
    uint8_t bus = 0;

    bool extended_mode = false;
    bool auto_poll = false;
    bool time_stamping = false;
    uint32_t poll_counter = 0;

    void printBusName() const;
    std::map<uint8_t, uint32_t> bus_speeds = {{0, 10000},  {1, 20000},  {2, 50000},
                                              {3, 100000}, {4, 125000}, {5, 250000},
                                              {6, 500000}, {7, 750000}, {8, 1000000}};
};

} // namespace piccante::lawicel