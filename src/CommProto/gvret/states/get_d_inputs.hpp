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
#include <ostream>
#include <utility>
#include "../gvret.hpp"


namespace piccante::gvret::state {
class get_d_inputs : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_d_inputs(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_D_INPUTS), out(host_out) {}

    Protocol enter() override {
        // no state required, send response and return to IDLE
        // TODO: Validate Response.
        uint8_t buffer[] = {
            GET_COMMAND, GET_D_INPUTS,
            /* TODO: getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) +
                   (getDigital(3) << 3) + (getDigital(4) << 4) + (getDigital(5) << 5) */
            0};
        // TODO: Checksum correct o.O shouldn't we use 3 bytes?
        // TODO: Originalcode txs a trailing 0... WTF is the trailing zero for?
        // Original Code also never actually sets anything in the buffer...
        // COLLIN!!!!!
        out << buffer << check_sum({buffer, sizeof(buffer)}) << std::flush;

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret