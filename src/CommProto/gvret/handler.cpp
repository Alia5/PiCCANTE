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
#include "handler.hpp"
#include <cstdint>
#include <pico/time.h>
#include "util/bin.hpp"
#include "proto.hpp"
#include "can2040.h"


namespace piccante::gvret {
bool handler::process_byte(uint8_t byte) {
    if (byte == ENABLE_BINARY_MODE) {
        binary_mode = true;
    }
    return protocol_fsm.tick(byte);
}

void handler::comm_can_frame(uint busnumber, const can2040_msg& frame) {
    if (!binary_mode) {
        return;
    }
    // TODO: MICROS as 4 byte value, but oh well... SavvyCAN only uses relative value
    // should be fine
    uint32_t const time = to_us_since_boot(get_absolute_time());

    auto id = frame.id & ~(CAN2040_ID_EFF | CAN2040_ID_RTR);

    constexpr size_t header_size =
        1 + 1 + 4 + 4 + 1; // GET_COMMAND + SEND_CAN_FRAME + time + id + dlc/bus

    // TODO: make class member.
    static std::vector<uint8_t> buf(header_size + 8 + 1);
    size_t pos = 0;

    buf[pos++] = GET_COMMAND;
    buf[pos++] = SEND_CAN_FRAME;
    std::memcpy(&buf[pos], &time, 4);
    pos += 4;
    std::memcpy(&buf[pos], &id, 4);
    pos += 4;
    buf[pos++] = static_cast<uint8_t>(frame.dlc + uint8_t(busnumber << 4));
    for (uint8_t i = 0; i < frame.dlc; i++) {
        buf[pos++] = frame.data[i];
    }
    buf[pos++] = 0;

    host_out.write(reinterpret_cast<const char*>(buf.data()), pos);
    host_out.flush();
}

void handler::set_binary_mode(bool mode) { binary_mode = mode; }

bool handler::get_binary_mode() const { return binary_mode; }

} // namespace gvret
