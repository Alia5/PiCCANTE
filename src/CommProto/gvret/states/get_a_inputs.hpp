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
#include "../../../util/bin.hpp"
#include "../gvret.hpp"
#include <cstdint>
#include <ostream>
#include <array>
#include <utility>

namespace piccante::gvret::state {
class get_a_inputs : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_a_inputs(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_A_INPUTS), out(host_out) {}
    Protocol enter() override {
        // no state required, send response and return to IDLE
        // TODO: Validate Response.
        // NOLINTNEXTLINE
        std::array<uint8_t, 16> buffer = {
            GET_COMMAND, GET_D_INPUTS, 0x0 & 0xFF, 0 >> 8, 0x0 & 0xFF, 0 >> 8,
            0x0 & 0xFF,  0 >> 8,       0x0 & 0xFF, 0 >> 8, 0x0 & 0xFF, 0 >> 8,
            0x0 & 0xFF,  0 >> 8,       0x0 & 0xFF, 0 >> 8,
            // TODO: just dummy data
        };
        // TODO: Checksum correct o.O?
        out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size()); // NOLINT
        // Original Code never set's the buffer used to calc checksum to anything...
        out << gvret::check_sum({buffer.data(), buffer.size()}) << std::flush;

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, true};
    }

        private:
    std::ostream& out;
};
} // namespace piccante::gvret::state
