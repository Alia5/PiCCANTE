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
#include "gvret.hpp"
#include <cstdint>
#include <pico/types.h>
#include <numeric>
#include "outstream/stream.hpp"
#include "../../CanBus/CanBus.hpp"
#include "can2040.h"
#include "proto.hpp"
#include "util/bin.hpp"
namespace piccante::gvret {

uint8_t check_sum(const std::span<uint8_t>& buff) {
    return std::accumulate(buff.begin(), buff.end(), 0);
}

void comm_can_frame(uint busnumber, const can2040_msg& frame, out::stream& out) {
    // TODO: can only cause trouble to output MICROS as 4 byte value, but oh well...
    // I don't think it is implemented anyway anywhere, just use millis for now...
    uint32_t const time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    auto id = frame.id;
    // remove flags from id copy
    id &= ~(CAN2040_ID_EFF | CAN2040_ID_RTR);


    out << GET_COMMAND << SEND_CAN_FRAME << piccante::bin(time) << piccante::bin(id)
        << static_cast<uint8_t>(frame.dlc + uint8_t(busnumber << 4));
    for (uint8_t i = 0; i < frame.dlc; i++) {
        out << frame.data[i];
    }
    // TODO: Checksum, seems not to be implemented anyway...
    out << uint8_t(0);
    out.flush();
}

} // namespace piccante::gvret