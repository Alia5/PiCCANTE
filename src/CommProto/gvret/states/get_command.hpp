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

#include "../../../Logger/Logger.hpp"
#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <span>
#include "../../../fmt.hpp"

namespace gvret {


namespace state {
class get_command : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_command() : fsm::state<uint8_t, Protocol, bool>(GET_COMMAND) {}
    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        // Protocol enum has gaps...
        switch (byte) {
            case SEND_CAN_FRAME:
                break;
            case TIME_SYNC:
                break;
            case GET_D_INPUTS:
                break;
            case GET_A_INPUTS:
                break;
            case SET_D_OUT:
                break;
            case SETUP_CANBUS_1_2:
                break;
            case GET_CANBUS_PARAMS_1_2:
                break;
            case GET_DEV_INFO:
                break;
            case SET_SW_MODE:
                break;
            case KEEPALIVE:
                break;
            case SET_SYSTYPE:
                break;
            case ECHO_CAN_FRAME:
                break;
            case GET_NUM_BUSES:
                break;
            case GET_CANBUS_PARAMS_3_4_5:
                break;
            case SET_CANBUS_PARAMS_3_4_5:
                break;
            case SEND_FD_FRAME:
                break;
            case SETUP_FD:
                break;
            case GET_FD_SETTINGS:
                break;

            case GET_COMMAND:
                Log::error << "Recieved get_command command, while in get_command mode, "
                              "resetting gvret::state\n";
                return {IDLE, false};
                break;
            case ENABLE_BINARY_MODE:
                Log::error << "Recieved enable binary mode command, while in binary "
                              "mode, resetting gvret::state\n";
                return {IDLE, false};
                break;
            case IDLE:
                Log::error << "Recieved idle command, resetting gvret::state\n";
                return {IDLE, false};
                break;
            default:
                Log::error << "Unknown command: " << byte << ", resetting gvret::state\n";
                return {IDLE, false};
        }

        return {static_cast<Protocol>(byte), false};
    }
};


} // namespace state
} // namespace gvret