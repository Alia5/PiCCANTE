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

#include "outstream/stream.hpp"
#include <vector>
#include <pico/time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace piccante::out {

class timeout_buffer_sink : public base_sink {
        public:
    explicit timeout_buffer_sink(base_sink& sink,
                                 uint32_t flush_timeout_ms = DEFAULT_FLUSH_TIMEOUT)
        : underlying_sink(sink), flush_timeout_ms(flush_timeout_ms) {
        buffer.reserve(BUFFER_SIZE);
        buffer_mutex = xSemaphoreCreateMutex();
        last_flush_time = to_ms_since_boot(get_absolute_time());
        xTaskCreate(
            flush_task_trampoline, fmt::sprintf("timeout_b_sink_%d", this).c_str(),
            configMINIMAL_STACK_SIZE, this, tskIDLE_PRIORITY + 1, &flush_task_handle);
    }

    ~timeout_buffer_sink() override {
        if (flush_task_handle != nullptr) {
            vTaskDelete(flush_task_handle);
            flush_task_handle = nullptr;
        }
        flush();
        if (buffer_mutex != nullptr) {
            vSemaphoreDelete(buffer_mutex);
            buffer_mutex = nullptr;
        }
    }

    void write(const char* data, std::size_t len) override {
        if (len > BUFFER_SIZE) {
            return;
        }
        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return;
        }
        if (buffer.size() + len > BUFFER_SIZE) {
            force_flush();
        }
        buffer.insert(buffer.end(), data, data + len);
        xSemaphoreGive(buffer_mutex);
    }

    void force_flush() {
        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return;
        }
        if (buffer.empty()) {
            xSemaphoreGive(buffer_mutex);
            return;
        }
        underlying_sink.write(buffer.data(), buffer.size());
        last_flush_time = to_ms_since_boot(get_absolute_time());
        underlying_sink.flush();
        buffer.clear();
        xSemaphoreGive(buffer_mutex);
    }

    void flush() override {
        const auto now = to_ms_since_boot(get_absolute_time());
        if ((now - last_flush_time) < flush_timeout_ms) {
            return;
        }
        force_flush();
    }

        private:
    base_sink& underlying_sink;
    std::vector<char> buffer{};
    uint32_t last_flush_time;
    uint32_t flush_timeout_ms;

    SemaphoreHandle_t buffer_mutex = nullptr;
    TaskHandle_t flush_task_handle = nullptr;

    static void flush_task_trampoline(void* param) {
        timeout_buffer_sink* self = static_cast<timeout_buffer_sink*>(param);
        self->flush_task();
    }

    void flush_task() {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (true) {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(flush_timeout_ms));

            const auto now = to_ms_since_boot(get_absolute_time());
            if ((now - last_flush_time) >= flush_timeout_ms) {
                force_flush();
            }
        }
    }

    static constexpr size_t BUFFER_SIZE = 1000;
    static constexpr uint32_t DEFAULT_FLUSH_TIMEOUT = 26;
};

} // namespace piccante::out