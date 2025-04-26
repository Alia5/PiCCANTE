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
extern "C" {
#include <can2040.h>
}
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
        response_queue = xQueueCreate(RESPONSE_QUEUE_SIZE, RESPONSE_QUEUE_ITEM_SIZE);
        if (!response_queue) {
            Log::error << "ELM327: Failed to create response queue\n";
        }
    };
    virtual ~emulator() {
        stop();
        if (response_queue) {
            vQueueDelete(response_queue);
            response_queue = nullptr;
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

    constexpr static uint64_t min_timeout_ms = 5;
    constexpr static uint64_t max_timeout_ms = 2555;


        private:
    out::stream& out;
    QueueHandle_t cmd_rx_queue;
    uint8_t bus;
    std::string last_command;

    QueueHandle_t response_queue;
    constexpr static uint8_t RESPONSE_QUEUE_SIZE = 32;
    constexpr static uint8_t RESPONSE_QUEUE_ITEM_SIZE = sizeof(can2040_msg);

    uint8_t expected_frames = 0;
    uint8_t received_frames = 0;

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

    struct PidTimeoutData {
        uint64_t avg_response_time;
        uint8_t sample_count;
        uint64_t last_update_time;
    };

    std::unordered_map<uint32_t, PidTimeoutData> pid_timing_history;

    uint32_t make_pid_key(uint8_t service, uint32_t pid) {
        return (static_cast<uint32_t>(service) << 24) | (pid & 0x00FFFFFF);
    }

    elm327::at at_h{
        out, params,
        std::bind(&emulator::reset, this, std::placeholders::_1, std::placeholders::_2)};

    bool is_waiting_for_response = false;
    bool processed_response = false;
    struct {
        uint8_t service;
        uint32_t pid;
        uint64_t request_start_time;
    } current_request;

    bool running = false;
    TaskHandle_t task_handle;

    void reset(bool settings, bool timeout);

    void update_timeout(uint64_t now, uint64_t response_time, uint8_t service,
                        uint32_t pid);
    void check_for_timeout();
    void cleanup_pid_history(uint64_t now);
    void process_input_byte(uint8_t byte, std::vector<char>& buff);
    void process_can_frame(const can2040_msg& frame);
    void format_frame_output(const can2040_msg& frame, uint32_t id,
                             std::string& outBuff) const;


    static inline bool is_capability_request(uint8_t service, uint32_t pid) {
        // Service 01, PIDs 00, 20, 40, 60, 80, A0, C0, E0
        if (service == 0x01 &&
            (pid == 0x00 || pid == 0x20 || pid == 0x40 || pid == 0x60 || pid == 0x80 ||
             pid == 0xA0 || pid == 0xC0 || pid == 0xE0)) {
            return true;
        }

        // Service 09, PIDs 00, 20, 40, 60, 80, etc.
        if (service == 0x09 && (pid % 0x20) == 0x00) {
            return true;
        }

        // Service 02, PIDs 00, 20, 40, etc. (similar to 01)
        if (service == 0x02 && (pid % 0x20) == 0x00) {
            return true;
        }

        return false;
    }

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