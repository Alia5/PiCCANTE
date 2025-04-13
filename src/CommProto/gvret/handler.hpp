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
#include "../../StateMachine/StateMachine.hpp"
#include "proto.hpp"
#include "states/states.hpp"

// My implementation of the gvret protocol (afaik) originally by Collin Kidder (Collin80)
// Based on the original code from:
// Protocol details (implementations can be found at:
// https://github.com/collin80/M2RET/blob/master/CommProtocol.txt
// esp32 arduino impl: https://github.com/collin80/ESP32RET/blob/master/gvret_comm.h
// SavvyCAN host impl:
// https://github.com/collin80/SavvyCAN/blob/master/connections/gvretserial.cpp
//
// check handler and states for more details
// binary protocol enum in proto.hpp

namespace piccante::gvret {

class handler {
        public:
    explicit handler(out::stream& out_stream) : host_out(out_stream) {};
    virtual ~handler() = default;

    // Disable copy and move operations
    handler(const handler&) = delete;
    handler& operator=(const handler&) = delete;
    handler(handler&&) = delete;
    handler& operator=(handler&&) = delete;

    bool process_byte(uint8_t byte);
    void comm_can_frame(uint busnumber, const can2040_msg& frame);
    void set_binary_mode(bool mode);
    bool get_binary_mode() const;

        private:
    out::stream& host_out;
    bool binary_mode = false;
    fsm::StateMachine<uint8_t, Protocol, bool> protocol_fsm{
        state::idle(),
        state::get_command(),
        state::build_can_frame(),
        state::time_sync(host_out),
        state::get_d_inputs(host_out),
        state::get_a_inputs(host_out),
        state::set_d_out(),
        state::setup_canbus_1_2(),
        state::get_canbus_params_1_2(host_out),
        state::get_dev_info(host_out),
        state::set_sw_mode(),
        state::keepalive(host_out),
        state::set_systype(),
        state::echo_can_frame([this](uint busnumber, const can2040_msg& frame) {
            this->comm_can_frame(busnumber, frame);
        }),
        state::get_num_buses(host_out),
        state::get_canbus_params_3_4_5(host_out),
        state::set_canbus_params_3_4_5(),
        // TODO: impl:
        state::send_fd_frame(),
        state::setup_fd(),
        state::get_fd_settings(),
    };
};


} // namespace gvret