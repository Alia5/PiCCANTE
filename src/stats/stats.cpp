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
#include "stats.hpp"
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "fs/littlefs_driver.hpp"
#include "lfs.h"

namespace piccante::sys::stats {

namespace {

struct TaskStatusInfo {
    TaskStatus_t status;
    std::string name; // Safe copy of the task name

    TaskStatusInfo(const TaskStatus_t& task_status)
        : status(task_status), name(task_status.pcTaskName) {}
};

TaskHandle_t stats_task_handle = nullptr;
SemaphoreHandle_t stats_mutex = nullptr;
std::vector<TaskStatusInfo> previous_snapshot;
std::vector<TaskStatusInfo> current_snapshot;
unsigned long previous_total_runtime = 0;
unsigned long current_total_runtime = 0;
bool stats_initialized = false;

static void stats_collection_task(void*) {
    constexpr size_t MAX_TASKS = 16;
    std::vector<TaskStatus_t> raw_snapshot(MAX_TASKS);
    std::vector<TaskStatusInfo> temp_snapshot;

    for (;;) {
        unsigned long temp_total_runtime = 0;
        UBaseType_t task_count =
            uxTaskGetSystemState(raw_snapshot.data(), MAX_TASKS, &temp_total_runtime);
        raw_snapshot.resize(task_count);
        temp_snapshot.clear();
        for (const auto& task : raw_snapshot) {
            if (task.pcTaskName == nullptr) {
                continue; // Skip tasks with no name
            }
            temp_snapshot.emplace_back(task);
        }

        if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            previous_snapshot = std::move(current_snapshot);
            previous_total_runtime = current_total_runtime;

            current_snapshot = std::move(temp_snapshot);
            current_total_runtime = temp_total_runtime;

            xSemaphoreGive(stats_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace

void init_stats_collection() {
    if (!stats_initialized) {
        stats_mutex = xSemaphoreCreateMutex();
        if (stats_mutex != nullptr) {
            xTaskCreate(stats_collection_task,
                        "StatsTask",
                        configMINIMAL_STACK_SIZE,
                        nullptr,
                        tskIDLE_PRIORITY + 1,
                        &stats_task_handle);
            stats_initialized = true;
        }
    }
}

std::size_t num_digits(unsigned long num) {
    if (num == 0) {
        return 1;
    }
    std::size_t count = 0;
    while (num > 0) {
        count++;
        num /= 10;
    }
    return count;
}

MemoryStats get_memory_stats() {
    MemoryStats stats{};

    stats.free_heap = xPortGetFreeHeapSize();
    stats.min_free_heap = xPortGetMinimumEverFreeHeapSize();
    stats.total_heap = configTOTAL_HEAP_SIZE;
    stats.heap_used = stats.total_heap - stats.free_heap;
    stats.heap_usage_percentage =
        static_cast<float>(stats.heap_used * 100) / stats.total_heap;

    return stats;
}

std::vector<TaskInfo> get_task_stats() {
    constexpr size_t MAX_TASKS = 16;
    std::vector<TaskStatus_t> task_status{MAX_TASKS};
    unsigned long total_runtime = 0;

    UBaseType_t task_count =
        uxTaskGetSystemState(task_status.data(), MAX_TASKS, &total_runtime);
    std::vector<TaskInfo> task_info;
    task_status.resize(task_count);

    for (const auto& task : task_status) {
        TaskInfo info{};

        info.name = task.pcTaskName;
        info.priority = task.uxCurrentPriority;
        info.stack_high_water = task.usStackHighWaterMark;
        info.handle = task.xHandle;
        info.task_number = task.xTaskNumber;
        info.core_affinity = task.uxCoreAffinityMask;
        info.runtime = task.ulRunTimeCounter;
        info.cpu_usage =
            (total_runtime > 0)
                ? static_cast<float>(task.ulRunTimeCounter * 100.0F / total_runtime)
                : 0.0F;

        info.state = task.eCurrentState;

        task_info.push_back(info);
    }

    return task_info;
}

std::vector<TaskInfo> get_cpu_stats(bool momentary) {
    if (!stats_initialized) {
        init_stats_collection();
        return {};
    }

    std::vector<TaskInfo> cpu_stats;

    if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return cpu_stats;
    }

    if (momentary && !previous_snapshot.empty()) {
        const auto runtime_diff = current_total_runtime - previous_total_runtime;

        if (runtime_diff > 0) {
            std::unordered_map<TaskHandle_t, const TaskStatusInfo*> prev_task_map;
            for (const auto& task : previous_snapshot) {
                prev_task_map[task.status.xHandle] = &task;
            }

            for (const auto& curr_task : current_snapshot) {
                auto prev_it = prev_task_map.find(curr_task.status.xHandle);
                if (prev_it != prev_task_map.end()) {
                    TaskInfo info{};
                    info.name = curr_task.name;
                    info.priority = curr_task.status.uxCurrentPriority;
                    info.stack_high_water = curr_task.status.usStackHighWaterMark;
                    info.handle = curr_task.status.xHandle;
                    info.task_number = curr_task.status.xTaskNumber;
                    info.core_affinity = curr_task.status.uxCoreAffinityMask;

                    const auto task_runtime = curr_task.status.ulRunTimeCounter -
                                              prev_it->second->status.ulRunTimeCounter;
                    info.runtime = task_runtime;
                    info.cpu_usage =
                        static_cast<float>(task_runtime * 100.0F / runtime_diff);
                    info.state = curr_task.status.eCurrentState;

                    cpu_stats.push_back(info);
                }
            }
        }
    } else {
        for (const auto& task : current_snapshot) {
            TaskInfo info{};
            info.name = task.name;
            info.priority = task.status.uxCurrentPriority;
            info.stack_high_water = task.status.usStackHighWaterMark;
            info.handle = task.status.xHandle;
            info.task_number = task.status.xTaskNumber;
            info.core_affinity = task.status.uxCoreAffinityMask;
            info.runtime = task.status.ulRunTimeCounter;
            info.cpu_usage = (current_total_runtime > 0)
                                 ? static_cast<float>(task.status.ulRunTimeCounter *
                                                      100.0f / current_total_runtime)
                                 : 0.0F;
            info.state = task.status.eCurrentState;

            cpu_stats.push_back(info);
        }
    }

    xSemaphoreGive(stats_mutex);
    return cpu_stats;
}

UptimeInfo get_uptime() {
    UptimeInfo info{};

    info.total_ticks = xTaskGetTickCount();
    const auto seconds = info.total_ticks / configTICK_RATE_HZ;

    info.days = seconds / 86400;
    info.hours = (seconds % 86400) / 3600;
    info.minutes = (seconds % 3600) / 60;
    info.seconds = seconds % 60;

    return info;
}

FilesystemStats get_filesystem_stats() {
    FilesystemStats stats{};

    stats.block_size = LFS_BLOCK_SIZE;
    stats.block_count = LFS_BLOCK_COUNT;
    stats.total_size = static_cast<size_t>(stats.block_size) * stats.block_count;

    const auto used_blocks = lfs_fs_size(&fs::lfs);
    if (used_blocks >= 0) {
        stats.used_size = static_cast<size_t>(used_blocks) * stats.block_size;
        stats.free_size = stats.total_size - stats.used_size;
        stats.usage_percentage =
            (static_cast<float>(stats.used_size) / stats.total_size) * 100.0f;
    }

    return stats;
}

} // namespace piccante::sys::stats