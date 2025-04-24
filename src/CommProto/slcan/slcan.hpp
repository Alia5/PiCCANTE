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

#include <cstdint>
#include "outstream/stream.hpp"
#include "FreeRTOS.h"
#include <task.h>
#include <map>
#include <vector>

struct can2040_msg;

namespace piccante::slcan {


enum SHORT_CMD : uint8_t {
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

enum LONG_CMD : uint8_t {
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
    TOGGLE_TIMESTAMP_ALT = 'Y',
    TOGGLE_AUTO_START = 'Q',
    CONFIGURE_BUS = 'C',
};


static inline std::map<uint8_t, uint32_t> bus_speeds = {
    {0, 10000},  {1, 20000},  {2, 50000},  {3, 100000}, {4, 125000},
    {5, 250000}, {6, 500000}, {7, 750000}, {8, 1000000}};
class handler {
        public:
    explicit handler(out::stream& out_stream, uint8_t read_itf, uint8_t bus_num = 0)
        : host_out(out_stream), read_itf(read_itf), bus(bus_num) {};
    virtual ~handler() = default;

    TaskHandle_t& create_task(UBaseType_t priority = configMAX_PRIORITIES - 10);

    void handle_short_cmd(char cmd);
    void handle_long_cmd(const std::string_view& cmd);
    void handle_command(const std::string_view& cmd);
    void comm_can_frame(const can2040_msg& frame);


        private:
    out::stream& host_out;
    uint8_t read_itf = 0;
    uint8_t bus = 0;

    TaskHandle_t task_handle = nullptr;

    bool extended_mode = false;
    bool auto_poll = true;
    bool time_stamping = false;
    uint32_t poll_counter = 0;

    std::vector<uint8_t> can_out_buffer{40};


    void printBusName() const;
    static void task_dispatcher(void* param);
    void task();
};

} // namespace piccante::slcan