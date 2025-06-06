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

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <utility>
#include <cstdint>

namespace piccante::gvret::state {
class set_sw_mode : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit set_sw_mode() : fsm::state<uint8_t, Protocol, bool>(SET_SW_MODE) {}
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        // Not supported, just go back to IDLE
        return {IDLE, false};
    }
};
} // namespace gvret