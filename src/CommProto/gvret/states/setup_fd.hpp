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
namespace piccante::gvret::state {
class setup_fd : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit setup_fd() : fsm::state<uint8_t, Protocol, bool>(SETUP_FD) {}
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        // TODO: Implementation
        return {IDLE, false};
    }
};
} // namespace piccante::gvret::state
