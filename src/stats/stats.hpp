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

#include <cstddef>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include "FreeRTOS.h"
#include "task.h"

namespace piccante::sys::stats {


struct MemoryStats {
    size_t free_heap;            ///< Current free heap size in bytes
    size_t min_free_heap;        ///< Minimum free heap size ever recorded in bytes
    size_t total_heap;           ///< Total heap size in bytes
    size_t heap_used;            ///< Currently used heap in bytes
    float heap_usage_percentage; ///< Percentage of heap currently used
};

struct TaskSwitchEvent {
    unsigned long timestamp;
    TaskHandle_t handle;
    uint8_t core_id;
    bool switched_out;
};

struct TaskRuntimeStats {
    unsigned long total_runtime;
    std::array<unsigned long, configNUM_CORES> runtime;
    std::array<unsigned long, configNUM_CORES> last_switch_in;
};

struct TaskInfo {
    std::string_view name;
    eTaskState state;
    UBaseType_t priority;
    uint16_t stack_high_water;
    TaskHandle_t handle;
    UBaseType_t task_number;
    UBaseType_t core_affinity;
    TaskRuntimeStats runtime;
    std::array<float, configNUM_CORES> cpu_usage;
    uint8_t core_id;
};

struct TaskStats {
    unsigned long total_runtime;
    std::array<float, configNUM_CORES> cores;
    std::vector<TaskInfo> tasks;
};

struct UptimeInfo {
    unsigned long days;     ///< Days
    unsigned long hours;    ///< Hours
    unsigned long minutes;  ///< Minutes
    unsigned long seconds;  ///< Seconds
    TickType_t total_ticks; ///< Total tick count
};

struct AdcStats {
    float value; // The processed measurement value
    uint16_t raw_value;
    uint8_t channel;
    std::string name;
    std::string unit;
};


void init_stats_collection();
MemoryStats get_memory_stats();
UptimeInfo get_uptime();

struct FilesystemStats {
    size_t total_size;      ///< Total size of filesystem in bytes
    size_t used_size;       ///< Used space in bytes
    size_t free_size;       ///< Free space in bytes
    float usage_percentage; ///< Used space as percentage
    int block_count;        ///< Total blocks
    int block_size;         ///< Block size in bytes
};
FilesystemStats get_filesystem_stats();

std::vector<AdcStats> get_adc_stats();

const TaskStats get_task_stats();

} // namespace piccante::sys::stats