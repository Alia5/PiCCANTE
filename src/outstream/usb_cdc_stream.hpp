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
#include <vector>
#include <memory>
#include "Logger/Logger.hpp"

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

inline std::vector<std::unique_ptr<USB_CDC_Sink>> sinks;
inline std::vector<std::unique_ptr<out::stream>> streams;

inline out::stream& out(uint8_t itf) {
    if (itf >= CFG_TUD_CDC) {
        Log::error << "Invalid USB CDC interface number: " << itf << "\n";
        return *streams[0];
    }
    while (itf >= streams.size()) {
        uint8_t new_idx = streams.size();
        sinks.push_back(std::make_unique<USB_CDC_Sink>(new_idx));
        streams.push_back(std::make_unique<out::stream>(*sinks.back()));
    }

    return *streams[itf];
}

inline out::stream& out0 = out(0);

} // namespace usb_cdc
