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
#include "../../../util/bin.hpp"
#include "outstream/stream.hpp"
#include "../../../Logger/Logger.hpp"
namespace piccante::gvret::state {
class get_canbus_params_1_2 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_canbus_params_1_2(out::stream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_CANBUS_PARAMS_1_2), out(host_out) {}

    Protocol enter() override {
        Log::debug << "get_canbus_params_1_2\n";
        // TODO: implement, for now just tx 500000kbits, enabled, NOT listen only
        const uint32_t speed = 500000;
        const uint8_t flags = 0x01 + (0x00 << 4); // enabled + listen only
        out << GET_COMMAND << GET_CANBUS_PARAMS_1_2 << flags << piccante::bin(speed)
            << flags << piccante::bin(speed);
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
} // namespace piccante::gvret::state