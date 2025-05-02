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
    uint32_t available = tud_cdc_n_write_available(itf);

    if (available == 0) {
        return;
    }

    uint32_t chunk_size = (s < available) ? s : available;
    uint32_t written = tud_cdc_n_write(itf, v, chunk_size);
    if (written == 0) {
        return;
    } else {
        retries = 0;
    }
    while (written < s && retries <= 10) {
        tud_cdc_n_write_flush(itf);
        taskYIELD();
        const size_t remaining = s - written;
        available = tud_cdc_n_write_available(itf);
        if (available > 0) {
            retries = 0;
            chunk_size = (remaining < available) ? remaining : available;
            written += tud_cdc_n_write(itf, v + written, chunk_size);
        } else {
            retries++;
            vTaskDelay(1);
            if (retries > 10) {
                break;
            }
        }
    }
}
void USB_CDC_Sink::flush() { tud_cdc_n_write_flush(itf); }
} // namespace usb_cdc
