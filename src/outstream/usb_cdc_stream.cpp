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
void USB_CDC_Sink::write(const char* v, std::size_t s) { tud_cdc_n_write(itf, v, s); }
void USB_CDC_Sink::flush() { tud_cdc_n_write_flush(itf); }
} // namespace usb_cdc
