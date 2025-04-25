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

#include <string_view>
#include <outstream/stream.hpp>
#include <optional>
#include <functional>
#include "settings.hpp"
#include "at_commands/commands.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "Logger/Logger.hpp"

struct can2040_msg;

#if defined bind
#undef bind
#endif

namespace piccante::elm327 {


class emulator {
        public:
    explicit emulator(out::stream& out, QueueHandle_t queue, uint8_t bus = 0)
        : out(out), cmd_rx_queue(queue), bus(bus) {
        state_mutex = xSemaphoreCreateMutex();
        if (!state_mutex) {
            Log::error << "ELM327: Failed to create mutex\n";
        }
    };
    virtual ~emulator() {
        stop();
        if (state_mutex) {
            vSemaphoreDelete(state_mutex);
            state_mutex = nullptr;
        }
    };

    emulator(const emulator&) = delete;
    emulator& operator=(const emulator&) = delete;

    bool start();
    void stop();
    bool is_running() const;

    void handle(std::string_view command);
    void handle_command(std::string_view command);
    void handle_pid_request(std::string_view command);

    void handle_can_frame(const can2040_msg& frame);

    constexpr static uint32_t obd2_11bit_broadcast = 0x7DF;
    constexpr static uint32_t obd2_29bit_broadcast = 0x18DB33F1;
    constexpr static std::string_view obdlink_desc = "ODBLink MX"; // TODO?
    constexpr static std::string_view device_desc = "PiCCANTE ELM327 Emulator";
    constexpr static std::string_view elm_id = "ELM327 v1.4"; // TODO:

    constexpr static uint64_t min_timeout_ms = 40;
    constexpr static uint64_t max_timeout_ms = 2500;
    uint64_t average_response_time_ms = 1000;

    constexpr static uint8_t SLOW_RESPONSE_WEIGHT =
        4; // How much weight to give new slow responses
    constexpr static uint8_t SLOW_RESPONSE_HISTORY_WEIGHT =
        6; // Weight for history with slow responses
    constexpr static uint8_t FAST_RESPONSE_WEIGHT =
        2; // How much weight to give new fast responses
    constexpr static uint8_t FAST_RESPONSE_HISTORY_WEIGHT =
        20; // Weight for history with fast responses

    constexpr static uint8_t TIMEOUT_SAFETY_FACTOR =
        3; // Multiply avg response time by this


        private:
    out::stream& out;
    QueueHandle_t cmd_rx_queue;
    uint8_t bus;
    std::string last_command;

    SemaphoreHandle_t state_mutex = nullptr;

    uint8_t expected_frames = 0;
    uint8_t received_frames = 0;
    constexpr static uint8_t MULTI_FRAME_TIMEOUT_MULTIPLIER =
        3; // Adjust based on testing

    elm327::settings params{
        .obd_header = obd2_11bit_broadcast,
        .timeout = 250,
        .line_feed = false,
        .echo = false,
        .white_spaces = true,
        .dlc = false,
        .monitor_mode = false,
        .memory = false,
        .print_headers = false,
        .use_extended_frames = false,
        .adaptive_timing = true,
    };


    elm327::at at_h{
        out, params,
        std::bind(&emulator::reset, this, std::placeholders::_1, std::placeholders::_2)};

    bool is_waiting_for_response = false;
    bool processed_response = false;
    uint64_t last_can_event_time = 0;

    bool running = false;
    TaskHandle_t task_handle;

    void reset(bool settings, bool timeout);

    void update_timeout(uint64_t now);
    void check_for_timeout();

    inline std::string_view end_ok() {
        if (params.line_feed) {
            return "\rOK\r\n>";
        }
        return "\rOK\r\r>";
    };
    static bool is_valid_hex(std::string_view str);
    static void emulator_task(void* params);
};

} // namespace piccante::elm327