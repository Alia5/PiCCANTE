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
#include <ostream>

#include "../../StateMachine/StateMachine.hpp"
#include "proto.hpp"
#include "states/states.hpp"

namespace piccante::gvret {

class handler {
        public:
    explicit handler(std::ostream& out_stream) : host_out(out_stream) {};
    virtual ~handler() = default;

    // Disable copy and move operations
    handler(const handler&) = delete;
    handler& operator=(const handler&) = delete;
    handler(handler&&) = delete;
    handler& operator=(handler&&) = delete;

    bool process_byte(uint8_t byte);

        private:
    std::ostream& host_out;
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
        state::echo_can_frame(host_out),
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