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
#include "settings.hpp"
#include <lfs.h>
#include "fs/littlefs_driver.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Logger/Logger.hpp"

namespace piccante::sys::settings {

namespace {
// Default settings
system_settings cfg = {
    //
    .echo = true,                                              //
    .log_level = static_cast<uint8_t>(Log::Level::LEVEL_INFO), //
    .led_mode = led::MODE_CAN,                                 //
};

namespace {
SemaphoreHandle_t settings_mutex = nullptr;

bool initialized = false;

constexpr auto SETTINGS_FILENAME = "sys_settings";
} // namespace

bool take_mutex(uint32_t timeout_ms = 100) {
    if (settings_mutex == nullptr) {
        settings_mutex = xSemaphoreCreateMutex();
        if (settings_mutex == nullptr) {
            Log::error << "Failed to create system settings mutex\n";
            return false;
        }
    }

    if (xSemaphoreTake(settings_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        Log::error << "Failed to take settings mutex\n";
        return false;
    }
    return true;
}

void give_mutex() {
    if (settings_mutex != nullptr) {
        xSemaphoreGive(settings_mutex);
    }
}

} // namespace

bool load_settings() {
    lfs_file_t readFile;
    const int err =
        lfs_file_open(&piccante::fs::lfs, &readFile, "system_settings", LFS_O_RDONLY);
    if (err == LFS_ERR_OK) {
        lfs_ssize_t const bytesRead =
            lfs_file_read(&piccante::fs::lfs, &readFile, &cfg, sizeof(cfg));
        if (bytesRead <= 0) {
            Log::error << "Failed to read system settings file\n";
            lfs_file_close(&piccante::fs::lfs, &readFile);
            return false;
        }
        lfs_file_close(&piccante::fs::lfs, &readFile);
    } else {
        Log::error << "Failed to read system settings file\n";
        return false;
    }
    initialized = true;
    return true;
}

const system_settings& get() {
    if (!initialized) {
        if (!load_settings()) {
            Log::error << "Failed to load system settings\n";
            return cfg;
        }
    }
    return cfg;
}

bool store() {
    if (!take_mutex()) {
        return false;
    }

    lfs_file_t writeFile;
    int err = lfs_file_open(&piccante::fs::lfs, &writeFile, SETTINGS_FILENAME,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    bool success = false;

    if (err == LFS_ERR_OK) {
        if (lfs_file_write(&piccante::fs::lfs, &writeFile, &cfg, sizeof(cfg)) > 0) {
            success = true;
        } else {
            Log::error << "Failed to write system settings file\n";
        }

        if (lfs_file_close(&piccante::fs::lfs, &writeFile) != LFS_ERR_OK) {
            Log::error << "Failed to close system settings file\n";
            success = false;
        }
    } else {
        Log::error << "Failed to open system settings file for writing\n";
    }

    give_mutex();
    return success;
}


bool get_echo() { return cfg.echo; }

void set_echo(bool enabled) { cfg.echo = enabled; }

uint8_t get_log_level() { return cfg.log_level; }

void set_log_level(uint8_t level) {
    cfg.log_level = level;
    Log::set_log_level(static_cast<Log::Level>(level));
}

void set_led_mode(led::Mode mode) {
    cfg.led_mode = mode;
    led::set_mode(mode);
}

led::Mode get_led_mode() { return led::Mode(); }

} // namespace piccante::sys::settings