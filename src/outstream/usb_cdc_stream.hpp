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

#include "stream.hpp"
#include <tusb.h>

namespace piccante::usb_cdc {
class USB_CDC_Sink : public out::base_sink {
        public:
    explicit USB_CDC_Sink(uint8_t itf) : itf(itf) {}
    USB_CDC_Sink() = default;
    void write(const char* v, std::size_t s) override;
    void flush() override;

        private:
    uint8_t itf = 0;
};


inline USB_CDC_Sink usb_cdc_sink0(0);
inline out::stream out0(usb_cdc_sink0);

inline USB_CDC_Sink usb_cdc_sink1(1);
inline out::stream out1(usb_cdc_sink1);

} // namespace usb_cdc
