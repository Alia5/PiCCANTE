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
#include "usb_cdc_stream.hpp"
#include "class/cdc/cdc_device.h"
#include <cstddef>

namespace piccante::usb_cdc {
void USB_CDC_Sink::write(const char* v, std::size_t s) {
    size_t remaining = s;
    const char* data = v;

    static auto retries = 0;
    constexpr auto max_retries = 1000;
    while (remaining > 0) {
        uint32_t available = tud_cdc_n_write_available(itf);

        if (available == 0) {
            // TODO: Cleanup buffer full hack.
            tud_cdc_n_write_flush(itf);
            if (++retries > max_retries) {
                retries = max_retries;
                // Log::error << "USB CDC write timeout on itf: " << itf
                //            << "; Discarding...\n";
                return;
            }
            if (available > 0) {
                retries = 0;
            }
            continue;
        }

        uint32_t chunk_size = (remaining < available) ? remaining : available;
        uint32_t written = tud_cdc_n_write(itf, data, chunk_size);

        data += written;
        remaining -= written;

        if (written < chunk_size) {
            tud_cdc_n_write_flush(itf);
        }
    }
}
void USB_CDC_Sink::flush() { tud_cdc_n_write_flush(itf); }
} // namespace usb_cdc
