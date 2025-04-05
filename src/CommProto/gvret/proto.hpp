/*
 * PiCCANTE - Pi Car Controller Area Network Tool for Exploration
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
namespace gvret {

enum Protocol : uint8_t {
    // CMDS
    SEND_CAN_FRAME = 0,
    TIME_SYNC = 1,
    GET_D_INPUTS = 2, // get Digial Inputs
    GET_A_INPUTS = 3, // Get Analog Inputs
    SET_D_OUT = 4,    // Set Digital Outputs
    SETUP_CANBUS_1_2 = 5,
    GET_CANBUS_PARAMS_1_2 = 6,
    GET_DEV_INFO = 7,
    SET_SW_MODE = 8,
    KEEPALIVE = 9,
    SET_SYSTYPE = 10,
    ECHO_CAN_FRAME = 11,
    GET_NUM_BUSES = 12,
    GET_CANBUS_PARAMS_3_4_5 = 13,
    SET_CANBUS_PARAMS_3_4_5 = 14,
    // Seems not implemented anywhere
    // We don't have support for CAN-FD anyway on default config.
    SEND_FD_FRAME = 20,
    SETUP_FD = 21,
    GET_FD_SETTINGS = 22,
    // BASE
    GET_COMMAND = 0xF1,
    ENABLE_BINARY_MODE = 0xE7,
    IDLE = 0xFF
};
}