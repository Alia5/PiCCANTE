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
#include "hardware/uart.h"

namespace piccante::uart {
class uart_sink : public out::base_sink {
        public:
    explicit uart_sink(uart_inst_t* uart_inst = uart0) : uart_instance(uart_inst) {}
    void init(uint8_t pin_tx, uint8_t pin_rx, uint32_t baud);
    void write(const char* v, std::size_t s) override;
    void flush() override;

        private:
    uart_inst_t* uart_instance;
};


inline uart_sink sink0{uart0};
inline out::stream out0{sink0};

} // namespace piccante::uart
