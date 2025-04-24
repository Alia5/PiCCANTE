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
#include <cstdint>
#include <utility>
#include "outstream/stream.hpp"
#include "../../../util/bin.hpp"

namespace piccante::gvret::state {
class keepalive : public fsm::state<uint8_t, Protocol, bool> {
    static constexpr uint16_t KEEPALIVE_PAYLOAD = 0xDEAD;

        public:
    explicit keepalive(out::stream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(KEEPALIVE), out(host_out) {}

    Protocol enter() override {
        // out << GET_COMMAND << KEEPALIVE << piccante::bin_be(KEEPALIVE_PAYLOAD);
        out << "\xf1\x09\xDE\xAD";
        out.flush();
        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    out::stream& out;
};
} // namespace gvret