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
#include "outstream/stream.hpp"
#include <utility>
#include "../../../util/bin.hpp"

namespace piccante::gvret::state {
class get_canbus_params_3_4_5 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_canbus_params_3_4_5(out::stream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_CANBUS_PARAMS_3_4_5), out(host_out) {}

    // TODO: doesn't seem to be implemented anywhere, just guessing response based on
    // other
    Protocol enter() override {
        // Max 3 busses supported, 4 and 5.. just copy params from 2 ¯\_(ツ)_/¯
        const uint8_t flags = can::is_enabled(2) + (can::is_listenonly(2) << 4);
        const auto speed = can::get_bitrate(2);
        out << GET_COMMAND << GET_CANBUS_PARAMS_3_4_5 << flags << piccante::bin(speed)
            << flags << piccante::bin(speed) << flags << piccante::bin(speed);
        out.flush();

        // TODO: doesn't TX a checksum byte ¯\_(ツ)_/¯

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    out::stream& out;
};
} // namespace gvret