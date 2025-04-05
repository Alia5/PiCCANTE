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
#include <FreeRTOS.h>
#include <cstdint>
#include <ostream>
#include "../../../util/bin.hpp"

namespace piccante::gvret::state {
class time_sync : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit time_sync(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(TIME_SYNC), out(host_out) {}
    Protocol enter() override {
        Log::info << "Time Sync\n" << std::flush;
        // no state required, send response and return to IDLE
        // TODO: can only cause trouble to output MICROS as 4 byte value, but oh well...
        // I don't think it is implemented anyway anywhere, just use millis for now...
        uint32_t time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        out << GET_COMMAND
            << TIME_SYNC
            // IS this really output in binary?
            << piccante::bin(time) << std::flush;
        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};


} // namespace gvret