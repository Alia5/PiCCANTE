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
#include "./stats.hpp"
#include "../../../stats/stats.hpp"
#include <string_view>
#include <string>
extern "C" {
#include "../server/httpserver.h"
}
#include "fmt.hpp"
#include "wifi/wifi.hpp"
#include "CanBus/CanBus.hpp"

namespace piccante::httpd::api::stats {
bool get(http_connection conn, [[maybe_unused]] std::string_view url) {
    auto* const handle =
        http_server_begin_write_reply(conn, "200 OK", "application/json", "");

    const auto uptime = sys::stats::get_uptime();
    const auto mem_stats = sys::stats::get_memory_stats();
    const auto cpu_stats = sys::stats::get_task_stats();
    const auto adc_stats = sys::stats::get_adc_stats();
    const auto fs_stats = sys::stats::get_filesystem_stats();
    const auto wifi_stats = wifi::wifi_stats();

    const auto avail_can = can::get_num_busses();


    const auto uptime_json = fmt::sprintf(                                //
        R"({"days":%d,"hours":%d,"minutes":%d,"seconds":%d,"ticks":%d})", //
        uptime.days, uptime.hours, uptime.minutes, uptime.seconds, uptime.total_ticks);

    const auto memory_json = fmt::sprintf(                                       //
        R"({"free_heap":%d,"min_free_heap":%d,"total_heap":%d,"heap_used":%d})", //
        mem_stats.free_heap, mem_stats.min_free_heap, mem_stats.total_heap,
        mem_stats.heap_used);

    auto cpu_json = fmt::sprintf(                                 //
        R"({"total_runtime":%d,"load0":%f,"load1":%f,"tasks":[)", //
        cpu_stats.total_runtime, cpu_stats.cores[0], cpu_stats.cores[1]);
    for (const auto& task : cpu_stats.tasks) {
        cpu_json += fmt::sprintf( //
            R"({"name":"%s","state":%d,"priority":%d,"stack_high_water":%d,"core_id":%d,"task_number":%d,"core_affinity":%d,)", //
            task.name.data(), static_cast<int>(task.state), task.priority,
            task.stack_high_water, task.core_id, task.task_number, task.core_affinity);

        for (size_t core = 0; core < configNUM_CORES; ++core) {
            cpu_json += fmt::sprintf(      //
                R"("cpu_usage_%d":%.2f,)", //
                core, task.cpu_usage[core]);
        }
        cpu_json.pop_back(); // Remove the last comma
        cpu_json += "},";
    }
    if (cpu_json.size() > 1) {
        cpu_json.pop_back(); // Remove the last comma
        cpu_json += "]}";
    }

    const auto fs_json = fmt::sprintf( //
        R"({"block_size":%d,"block_count":%d,"total_size":%d,"used_size":%d,"free_size":%d})", //
        fs_stats.block_size, fs_stats.block_count, fs_stats.total_size,
        fs_stats.used_size, fs_stats.free_size);

    std::string adc_json = "[";
    for (const auto& adc : adc_stats) {
        adc_json += fmt::sprintf(                                             //
            R"({"channel":%d,"value":%f,"raw":%d,"name":"%s","unit":"%s"},)", //
            adc.channel, adc.value, adc.raw_value, adc.name.c_str(), adc.unit.c_str());
    }
    if (adc_json.size() > 1) {
        adc_json.pop_back(); // Remove the last comma
    }
    adc_json += "]";

    std::string can_json = "[";
    for (size_t i = 0; i < avail_can; ++i) {
        const auto enabled = can::is_enabled(i);
        const auto bitrate = can::get_bitrate(i);
        const auto rx_buffered = can::get_can_rx_buffered_frames(i);
        const auto tx_buffered = can::get_can_tx_buffered_frames(i);
        const auto rx_overflow = can::get_can_rx_overflow_count(i);
        const auto tx_overflow = can::get_can_tx_overflow_count(i);

        can_json += fmt::sprintf( //
            R"({"bus":%d,"enabled":%s,"bitrate":%d,"rx_buffered":%d,"tx_buffered":%d,"rx_overflow":%d,"tx_overflow":%d,)", //
            static_cast<int>(i), enabled ? "true" : "false", bitrate, rx_buffered,
            tx_buffered, rx_overflow, tx_overflow);

        can2040_stats stats{};
        if (can::get_statistics(i, stats)) {
            can_json += fmt::sprintf(                                                //
                R"("rx_total":%d,"tx_total":%d,"tx_attempt":%d,"parse_error":%d},)", //
                stats.rx_total, stats.tx_total, stats.tx_attempt, stats.parse_error);
        } else {
            can_json.pop_back(); // Remove the last comma
            can_json += "},";
        }
    }
    if (can_json.size() > 1) {
        can_json.pop_back(); // Remove the last comma
    }
    can_json += "]";

    std::string wifi_json = "{}";
    if (wifi_stats) {
        wifi_json = fmt::sprintf( //
            R"({"mode":%d,"connected":%s,"ssid":"%s","channel":%d,"rssi":%d,"ip_address":"%s","mac_address":"%s"})", //
            static_cast<int>(wifi_stats->mode), wifi_stats->connected ? "true" : "false",
            wifi_stats->ssid.c_str(), static_cast<int>(wifi_stats->channel),
            wifi_stats->rssi, wifi_stats->ip_address.c_str(),
            wifi_stats->mac_address.c_str());
    }

    const auto response = fmt::sprintf( //
        R"({"is_pico2":%s,"uptime":%s,"memory":%s,"cpu":%s,"fs":%s,"adc":%s,"can":%s,"wifi":%s})", //
#if defined(PICO_RP2350)
        "true", //
#else
        "false", //
#endif
        uptime_json.c_str(), //
        memory_json.c_str(), //
        cpu_json.c_str(),    //
        fs_json.c_str(),     //
        adc_json.c_str(),    //
        can_json.c_str(),    //
        wifi_json.c_str()    //
    );

    http_server_write_raw(handle, response.c_str(), response.size());
    http_server_end_write_reply(handle, "");

    return true;
}
} // namespace piccante::httpd::api::stats