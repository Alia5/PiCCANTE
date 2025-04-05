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

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <utility>
#include <cstdint>
#include <ostream>

namespace piccante::gvret::state {
class get_num_buses : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_num_buses(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_NUM_BUSES), out(host_out) {}

    constexpr static uint8_t NUM_BUSES = 1; // TODO: get from config/build-config
    Protocol enter() override {
        out << GET_COMMAND << GET_NUM_BUSES << NUM_BUSES << std::flush;
        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret