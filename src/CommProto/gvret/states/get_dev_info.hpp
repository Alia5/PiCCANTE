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
#include <ostream>
#include "../../../util/bin.hpp"

namespace gvret::state {
class get_dev_info : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_dev_info(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_DEV_INFO), out(host_out) {}

    Protocol enter() override {
        // TODO: build number
        uint16_t build_number = 0x0001;
        out << GET_COMMAND << GET_DEV_INFO
            << piccante::bin(build_number)
            // TODO: Wtf is 0x20?!
            << (uint8_t(0x20))
            // TODO: WTF Is 0?
            << (uint8_t(0))
            // single wire mode?
            << (uint8_t(0)) << std::flush;

        // TODO: doesn't TX a checksum byte ¯\_(ツ)_/¯

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret