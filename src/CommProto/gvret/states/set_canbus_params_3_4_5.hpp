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
#include "SysShell/settings.hpp"

namespace piccante::gvret::state {
class set_canbus_params_3_4_5 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit set_canbus_params_3_4_5()
        : fsm::state<uint8_t, Protocol, bool>(SET_CANBUS_PARAMS_3_4_5) {}
    Protocol enter() override {
        step = 0;
        return SET_CANBUS_PARAMS_3_4_5;
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
                        can::set_listenonly(2, listen_only);
                        if (enabled) {
                            const auto& cfg = sys::settings::get();
                            if (cfg.baudrate_lockout) {
                                can::enable(2, can::get_bitrate(2));
                            } else {
                                can::enable(2, bus_speed);
                            }

                        } else {
                            can::disable(2);
                        }
                    } else {
                        const auto& cfg = sys::settings::get();
                        if (cfg.baudrate_lockout) {
                            can::enable(2, can::get_bitrate(2));
                        } else {
                            can::enable(2, can::DEFAULT_BUS_SPEED);
                        }
                        // Legacy behavior
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
                // ignore, not supported.
                break;
            case 8:
                buff_int = byte;
                break;
            case 9:
                buff_int |= byte << 8;
                break;
            case 10:
                buff_int |= byte << 16;
                break;
            case 11:
                buff_int |= byte << 24;
                bus_speed = buff_int & SPEED_MASK;
                // ignore, not supported.
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