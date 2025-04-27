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
#include <hardware/adc.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <projdefs.h>
#include <portable.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "pico/multicore.h"

#ifdef WIFI_ENABLED
#include "pico/cyw43_arch.h"
#endif


#include "fs/littlefs_driver.hpp"
#include "lfs.h"

#include "Logger/Logger.hpp"

namespace piccante::sys::stats {

namespace {
bool stats_initialized = false;
TaskHandle_t stats_task_handle = nullptr;

std::unordered_map<TaskHandle_t, TaskInfo> task_info;

SemaphoreHandle_t update_mutex = nullptr;

void populate_task_stats() {
    static std::vector<TaskStatus_t> task_states;
    static unsigned long total_runtime;

    static std::array<unsigned long, configNUM_CORES> core_runtimes = {};
    static std::array<unsigned long, configNUM_CORES> last_core_runtimes = {};
    static std::unordered_map<TaskHandle_t, std::array<unsigned long, configNUM_CORES>>
        last_task_runtimes;

    const auto num_tasks = uxTaskGetNumberOfTasks();
    task_states.resize(num_tasks);
    const auto tasks =
        uxTaskGetSystemState(task_states.data(), num_tasks, &total_runtime);

    // Reset core_runtimes
    for (size_t core = 0; core < configNUM_CORES; core++) {
        core_runtimes[core] = 0;
    }

    // Mark all tasks for potential deletion and accumulate runtimes
    for (auto& pair : task_info) {
        pair.second.task_number = static_cast<UBaseType_t>(-1);
        for (size_t core = 0; core < configNUM_CORES; core++) {
            core_runtimes[core] += pair.second.runtime.runtime[core];
        }
    }

    // Process all tasks from the system state
    for (const auto& task : task_states) {
        auto& info = task_info[task.xHandle];
        info.name = task.pcTaskName;
        info.state = task.eCurrentState;
        info.priority = task.uxCurrentPriority;
        info.stack_high_water = task.usStackHighWaterMark;
        info.task_number = task.xTaskNumber;
        info.core_affinity = task.uxCoreAffinityMask;

        if (last_task_runtimes.find(task.xHandle) == last_task_runtimes.end()) {
            last_task_runtimes[task.xHandle].fill(0);
        }

        for (size_t core = 0; core < configNUM_CORES; core++) {
            unsigned long core_delta = core_runtimes[core] - last_core_runtimes[core];
            if (core_delta > 10) { // Avoid noise
                unsigned long task_runtime_delta =
                    info.runtime.runtime[core] - last_task_runtimes[task.xHandle][core];
                info.cpu_usage[core] =
                    (static_cast<float>(task_runtime_delta) / core_delta) * 100.0f;
            } else {
                info.cpu_usage[core] = 0.0f;
            }
            last_task_runtimes[task.xHandle][core] = info.runtime.runtime[core];
        }
    }

    last_core_runtimes = core_runtimes;

    // for (const auto& pair : task_info) {
    //     if (pair.second.task_number == static_cast<UBaseType_t>(-1)) {
    //         task_info.erase(pair.second.handle);
    //         // Also clean up the runtime history
    //         last_task_runtimes.erase(pair.second.handle);
    //     }
    // }
    // Safe way to delete tasks: collect keys first, then erase
    std::vector<TaskHandle_t> tasks_to_delete;
    for (const auto& pair : task_info) {
        if (pair.second.task_number == static_cast<UBaseType_t>(-1)) {
            tasks_to_delete.push_back(pair.first);
        }
    }


    // Now delete the tasks
    for (const auto& handle : tasks_to_delete) {
        task_info.erase(handle);
        // Also clean up the runtime history
        last_task_runtimes.erase(handle);
    }
}

void stats_task(void* /*parameters*/) {
    TaskSwitchEvent event{};

    update_mutex = xSemaphoreCreateMutex();
    if (update_mutex == nullptr) {
        piccante::Log::error << "Failed to create task info mutex\n";
        return;
    }

    xSemaphoreTake(update_mutex, portMAX_DELAY);

    const auto populate_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
    populate_task_stats();
    xSemaphoreGive(update_mutex);
    stats_initialized = true;

    for (;;) {
        xSemaphoreTake(update_mutex, portMAX_DELAY);

        const auto populate_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
        populate_task_stats();
        xSemaphoreGive(update_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace
void init_stats_collection() {
    if (!stats_initialized) {
        xTaskCreate(stats_task, "StatsTask", configMINIMAL_STACK_SIZE, nullptr,
                    tskIDLE_PRIORITY + 1, &stats_task_handle);
    }
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


std::vector<AdcStats> get_adc_stats() {
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_initialized = true;
    }

    std::vector<AdcStats> results;

    // ADC0 (GPIO26) - Usually general purpose ADC
    adc_gpio_init(26);
    adc_select_input(0);
    uint16_t raw0 = adc_read();
    float voltage0 = (raw0 * 3.3f) / 4095.0f;
    results.push_back({.value = voltage0,
                       .raw_value = raw0,
                       .channel = 0,
                       .name = "ADC0",
                       .unit = "V"});

    // ADC1 (GPIO27) - Usually general purpose ADC
    adc_gpio_init(27);
    adc_select_input(1);
    uint16_t raw1 = adc_read();
    float voltage1 = (raw1 * 3.3f) / 4095.0f;
    results.push_back({.value = voltage1,
                       .raw_value = raw1,
                       .channel = 1,
                       .name = "ADC1",
                       .unit = "V"});

    // ADC2 (GPIO28) - Usually general purpose ADC
    adc_gpio_init(28);
    adc_select_input(2);
    uint16_t raw2 = adc_read();
    float voltage2 = (raw2 * 3.3f) / 4095.0f;
    results.push_back({.value = voltage2,
                       .raw_value = raw2,
                       .channel = 2,
                       .name = "ADC2",
                       .unit = "V"});

// ADC3 (GPIO29) - Connected to VSYS/3
#ifdef WIFI_ENABLED
    // This is a Pico W with WiFi/BT enabled - GPIO29 is shared with CYW43
    // To safely read VSYS, we need to set GPIO25 HIGH first
    // https://github.com/orgs/micropython/discussions/10421#discussioncomment-4609576

    // Save current state of GPIO25 (SPI CS pin)
    const bool old_gpio25_state = cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);

    // Set GPIO25 HIGH to enable GPIO29 for ADC use
    cyw43_arch_gpio_put(CYW43_WL_GPIO_VBUS_PIN, true);
    // Small delay to allow pin state to settle
    vTaskDelay(pdMS_TO_TICKS(1));

    // Now safe to read ADC3
    adc_gpio_init(29);
    adc_select_input(3);
    uint16_t raw3 = adc_read();
    float voltage3 = (raw3 * 3.3f) / 4095.0f;
    float vsys = voltage3 * 3.0f; // VSYS has 1:3 voltage divider

    // Restore GPIO25 to its previous state
    cyw43_arch_gpio_put(CYW43_WL_GPIO_VBUS_PIN, old_gpio25_state);

    results.push_back({.value = vsys,
                       .raw_value = raw3,
                       .channel = 3,
                       .name = "System Voltage",
                       .unit = "V"});
#else
    // This is a regular Pico - we can safely read ADC3
    adc_gpio_init(29);
    adc_select_input(3);
    uint16_t raw3 = adc_read();
    float voltage3 = (raw3 * 3.3f) / 4095.0f;
    float vsys = voltage3 * 3.0f; // VSYS has 1:3 voltage divider
    results.push_back({.value = vsys,
                       .raw_value = raw3,
                       .channel = 3,
                       .name = "System Voltage",
                       .unit = "V"});
#endif

    // ADC4 - Internal temperature sensor
    adc_set_temp_sensor_enabled(true);
    vTaskDelay(1); // Brief pause to let sensor stabilize

    constexpr int num_samples = 8;
    uint32_t temp_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        adc_select_input(4);
        temp_sum += adc_read();
        vTaskDelay(10);
    }

    uint16_t raw4 = temp_sum / num_samples;
    float voltage4 = (raw4 * 3.3f) / 4095.0f;

    // based on typical RP2040/RP2350 calibration
    // T = 27 - (ADC_voltage - 0.706)/0.001721
    float temp_c = 27.0f - (voltage4 - 0.706f) / 0.001721f;

    results.push_back({.value = temp_c,
                       .raw_value = raw4,
                       .channel = 4,
                       .name = "CPU Temperature",
                       .unit = "°C"});

    adc_set_temp_sensor_enabled(false);
    return results;
}

const TaskStats get_task_stats() {
    TaskStats stats{};
    stats.total_runtime = time_us_64();
    stats.tasks.reserve(task_info.size());

    xSemaphoreTake(update_mutex, portMAX_DELAY);

    for (const auto& pair : task_info) {
        const auto& tinfo = pair.second;
        stats.tasks.push_back(tinfo);
        if (tinfo.name.find("IDLE") != std::string::npos) {
            continue;
        }
        for (size_t core = 0; core < configNUM_CORES; core++) {
            stats.cores[core] += tinfo.cpu_usage[core];
        }
    }
    xSemaphoreGive(update_mutex);

    return stats;
}

} // namespace piccante::sys::stats

extern "C" void taskSwitchedIn(void* pxTask) {
    using namespace piccante::sys::stats;
    if (pxTask == nullptr || !stats_initialized) {
        return;
    }
    const auto crit = taskENTER_CRITICAL_FROM_ISR();

    const auto handle = static_cast<TaskHandle_t>(pxTask);
    const auto core_id = get_core_num();

    auto it = task_info.find(handle);
    if (it != task_info.end()) {
        auto& tinfo = it->second;
        tinfo.core_id = core_id;
        tinfo.runtime.last_switch_in[core_id] = time_us_64();
    }
    taskEXIT_CRITICAL_FROM_ISR(crit);
}
extern "C" void taskSwitchedOut(void* pxTask) {
    using namespace piccante::sys::stats;
    if (pxTask == nullptr || !stats_initialized) {
        return;
    }

    const auto handle = static_cast<TaskHandle_t>(pxTask);
    const auto core_id = get_core_num();

    const auto crit = taskENTER_CRITICAL_FROM_ISR();
    auto it = task_info.find(handle);
    if (it != task_info.end()) {
        auto& tinfo = it->second;
        tinfo.core_id = core_id;
        const auto diff = time_us_64() - tinfo.runtime.last_switch_in[core_id];
        tinfo.runtime.total_runtime += diff;
        tinfo.runtime.runtime[core_id] += diff;
    }
    taskEXIT_CRITICAL_FROM_ISR(crit);
}
