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
#include "st_commands/commands.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "CanBus/frame.hpp"
#include "Logger/Logger.hpp"


#if defined bind
#undef bind
#endif

namespace piccante::elm327 {


class emulator {
        public:
    explicit emulator(out::stream& out, QueueHandle_t queue, uint8_t bus = 0)
        : out(out), cmd_rx_queue(queue), bus(bus) {
        mtx = xSemaphoreCreateMutex();
        if (mtx == nullptr) {
            Log::error << "ELM327: Failed to create mutex\n";
        };
        outBuff.reserve(256);
    }
    virtual ~emulator() {
        stop();
        if (mtx) {
            vSemaphoreDelete(mtx);
            mtx = nullptr;
        }
    };

    emulator(const emulator&) = delete;
    emulator& operator=(const emulator&) = delete;

    bool start();
    void stop();
    bool is_running() const;

    int handle(std::string_view command);
    void handle_command(std::string_view command);
    int handle_pid_request(std::string_view command);

    void handle_can_frame(const can::frame& frame);

    constexpr static uint32_t obd2_11bit_broadcast = 0x7DF;
    constexpr static uint32_t obd2_29bit_broadcast = 0x18DB33F1;
    constexpr static std::string_view obdlink_desc = "ODBLink MX"; // TODO?
    constexpr static std::string_view device_desc = "PiCCANTE ELM327 Emulator";
    constexpr static std::string_view elm_id = "ELM327 v1.4"; // TODO:

    inline bool is_valid_hex(std::string_view str) {
        return std::all_of(str.begin(), str.end(), [](char byte) {
            return (byte >= '0' && byte <= '9') || (byte >= 'A' && byte <= 'F') ||
                   (byte >= 'a' && byte <= 'f');
        });
    }

        private:
    out::stream& out;
    QueueHandle_t cmd_rx_queue;
    uint8_t bus;
    std::string last_command;
    std::string outBuff{};


    SemaphoreHandle_t mtx = nullptr;

    elm327::settings params{
        .obd_header = obd2_11bit_broadcast,
        .timeout = 500,
        .line_feed = false,
        .echo = true,
        .white_spaces = true,
        .dlc = false,
        .monitor_mode = false,
        .memory = false,
        .print_headers = false,
        .use_extended_frames = false,
        .adaptive_timing = true,
    };

    bool is_waiting_for_response = false;
    struct {
        uint64_t last_response_time = 0;
        uint64_t request_start_time;
        uint32_t pid;
        uint8_t service;
        uint8_t received_frames = 0;
        uint8_t return_after_n_frames = 0;
        bool processed_response = false;
    } current_request;

    elm327::at at_h{
        out, params,
        std::bind(&emulator::reset, this, std::placeholders::_1, std::placeholders::_2)};
    elm327::st st_h{
        out, params,
        std::bind(&emulator::reset, this, std::placeholders::_1, std::placeholders::_2)};

    bool running = false;
    TaskHandle_t task_handle;

    void reset(bool settings, bool timeout);

    void update_timeout(uint64_t now, uint64_t response_time, uint8_t service,
                        uint32_t pid);
    int check_for_timeout();
    int process_input_byte(uint8_t byte, std::vector<char>& buff);
    void process_can_frame(const can::frame& frame);
    void format_frame_output(const can::frame& frame, uint32_t id,
                             std::string& outBuff) const;
    std::string_view end_ok() {
        if (params.line_feed) {
            return "OK\r\n>";
        }
        return "OK\r\r>";
    };

    static void emulator_task(void* params);

    static constexpr char HEX_CHARS[] = "0123456789ABCDEF";
};

namespace iso_tp {
enum FrameType : uint8_t {
    FirstFrame = 0x10,
    ConsecutiveFrame = 0x20,
    FlowControl = 0x30,
};
}

} // namespace piccante::elm327