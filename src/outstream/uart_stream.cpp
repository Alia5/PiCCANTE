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
#include "uart_stream.hpp"
#include <cstddef>
#include <cstdint>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"


namespace piccante::uart {

void uart_sink::init(uint8_t pin_tx, uint8_t pin_rx, uint32_t baud) {
    uart_init(uart_instance, baud);

    gpio_set_function(pin_tx, UART_FUNCSEL_NUM(uart_instance, pin_tx));
    gpio_set_function(pin_rx, UART_FUNCSEL_NUM(uart_instance, pin_rx));
}

void uart_sink::write(const char* v, std::size_t s) {
    uart_write_blocking(uart_instance, reinterpret_cast<const uint8_t*>(v), s);
}

void uart_sink::flush() {
    // Wait until UART has finished sending all data
    uart_tx_wait_blocking(uart_instance);
}
} // namespace piccante::uart