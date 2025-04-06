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

#include <cstdint>
#include <algorithm>
#include <utility>
#include "outstream/stream.hpp"
#include <cstdint>
#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include "../gvret.hpp"
#include "CanBus/CanBus.hpp"

namespace piccante::gvret::state {
class setup_canbus_1_2 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit setup_canbus_1_2() : fsm::state<uint8_t, Protocol, bool>(SETUP_CANBUS_1_2) {}
    Protocol enter() override {
        step = 0;
        return SETUP_CANBUS_1_2;
    }

    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        switch (step) { // NOLINT
            case 0:
                buff_int = byte;
                break;
            case 1:
                buff_int |= byte << 8U;
                break;
            case 2:
                buff_int |= byte << 16U;
                break;
            case 3:
                buff_int |= byte << 24U;
                bus_speed = buff_int & SPEED_MASK;
                if (bus_speed > MAX_BUS_SPEED) {
                    bus_speed = MAX_BUS_SPEED;
                }

                if (buff_int == 0) {
                    can::disable(0);
                } else {
                    if (buff_int & BUS_FLAG_MASK) {
                        bool enabled = buff_int & ENABLED_MASK;
                        bool listen_only = buff_int & LISTEN_ONLY_MASK;
                        can::set_listenonly(0, listen_only);
                        if (enabled) {
                            can::enable(0, bus_speed);
                        } else {
                            can::disable(0);
                        }
                    } else {
                        // Legacy behavior
                        can::enable(0, can::DEFAULT_BUS_SPEED);
                    }
                }
                break;
            case 4:
                buff_int = byte;
                break;
            case 5:
                buff_int |= byte << 8;
                break;
            case 6:
                buff_int |= byte << 16;
                break;
            case 7:
                buff_int |= byte << 24;
                bus_speed = buff_int & SPEED_MASK;
                if (bus_speed > MAX_BUS_SPEED) {
                    bus_speed = MAX_BUS_SPEED;
                }

                if (buff_int == 0) {
                    can::disable(1);
                } else {
                    if (buff_int & BUS_FLAG_MASK) {
                        bool enabled = buff_int & ENABLED_MASK;
                        bool listen_only = buff_int & LISTEN_ONLY_MASK;
                        can::set_listenonly(1, listen_only);
                        if (enabled) {
                            can::enable(1, bus_speed);
                        } else {
                            can::disable(1);
                        }
                    } else {
                        // Legacy behavior
                        can::enable(1, can::DEFAULT_BUS_SPEED);
                    }
                }
                return {IDLE, true};
        }
        step++;
        return {SETUP_CANBUS_1_2, true};
    }

        protected:
    uint8_t step = 0;
    uint32_t buff_int = 0;
    uint32_t bus_speed = 0;
};
} // namespace piccante::gvret::state