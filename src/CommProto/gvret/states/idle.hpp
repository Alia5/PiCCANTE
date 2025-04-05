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
#include <cstdint>
#include <utility>
namespace gvret::state {

class idle : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit idle() : fsm::state<uint8_t, Protocol, bool>(IDLE) {}

    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        if (byte == GET_COMMAND) {
            return {GET_COMMAND, true};
        }
        if (byte == ENABLE_BINARY_MODE) {
            return {IDLE, true};
        }
        return {IDLE, false};
    }
};

}