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
#include <array>
#include <algorithm>
#include <string>
#include <vector>
#include <cstdint>
#include <lfs.h>
#include "fs/littlefs_driver.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Logger/Logger.hpp"

namespace piccante::sys::settings {

namespace {
system_settings cfg = {
    //
    .echo = true,                                              //
    .log_level = static_cast<uint8_t>(Log::Level::LEVEL_INFO), //
    .led_mode = led::MODE_CAN,
    .wifi_mode = 0, //
};

#ifdef WIFI_ENABLED
wifi_settings wifi_cfg = {
    .ssid{},
    .password{},
    .channel = 1,
};
#endif

SemaphoreHandle_t settings_mutex = nullptr;

bool initialized = false;

constexpr auto SETTINGS_FILENAME = "system_settings";
constexpr auto WIFI_DATA_FILENAME = "wifi_data";

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
    int err =
        lfs_file_open(&piccante::fs::lfs, &readFile, SETTINGS_FILENAME, LFS_O_RDONLY);
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
#ifdef WIFI_ENABLED
    std::array<char, 128> wifi_buffer{};
    lfs_ssize_t bytesRead = 0;
    err = lfs_file_open(&piccante::fs::lfs, &readFile, WIFI_DATA_FILENAME, LFS_O_RDONLY);
    if (err == LFS_ERR_OK) {
        bytesRead = lfs_file_read(&piccante::fs::lfs, &readFile, wifi_buffer.data(),
                                  wifi_buffer.size());
        if (bytesRead <= 0) {
            Log::error << "Failed to read wifi settings file\n";
            lfs_file_close(&piccante::fs::lfs, &readFile);
            return false;
        }
        lfs_file_close(&piccante::fs::lfs, &readFile);
    } else {
        Log::error << "Failed to read wifi settings file\n";
        return false;
    }
    if (bytesRead > 0) {
        wifi_cfg.channel = wifi_buffer[0];

        size_t pos_term = 0;
        for (size_t i = 1; i < bytesRead; i++) {
            if (wifi_buffer[i] == 0) {
                pos_term = i;
                break;
            }
        }

        if (pos_term == 0) {
            Log::error << "Invalid wifi settings file: missing SSID terminator\n";
            return false;
        }

        size_t pos_pwd_term = 0;
        for (size_t i = pos_term + 1; i < bytesRead; i++) {
            if (wifi_buffer[i] == 0) {
                pos_pwd_term = i;
                break;
            }
        }

        if (pos_pwd_term == 0) {
            Log::error << "Invalid wifi settings file: missing password terminator\n";
            return false;
        }
        size_t ssid_len = pos_term - 1;
        wifi_cfg.ssid.resize(ssid_len);
        if (ssid_len > 0) {
            memcpy(&wifi_cfg.ssid[0], &wifi_buffer[1], ssid_len);
        }

        size_t pwd_len = pos_pwd_term - pos_term - 1;
        wifi_cfg.password.resize(pwd_len);
        if (pwd_len > 0) {
            memcpy(&wifi_cfg.password[0], &wifi_buffer[pos_term + 1], pwd_len);
        }

        Log::debug << "Loaded SSID: '" << wifi_cfg.ssid << "'\n";
    }

#endif

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
    auto err = lfs_file_open(&piccante::fs::lfs, &writeFile, SETTINGS_FILENAME,
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

#ifdef WIFI_ENABLED
    err = lfs_file_open(&piccante::fs::lfs, &writeFile, WIFI_DATA_FILENAME,
                        LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    success = false;

    if (err == LFS_ERR_OK) {
        std::vector<uint8_t> wifi_buffer;
        wifi_buffer.push_back(wifi_cfg.channel);

        for (size_t i = 0; i < wifi_cfg.ssid.size(); i++) {
            wifi_buffer.push_back(static_cast<uint8_t>(wifi_cfg.ssid[i]));
        }
        wifi_buffer.push_back(0);

        for (size_t i = 0; i < wifi_cfg.password.size(); i++) {
            wifi_buffer.push_back(static_cast<uint8_t>(wifi_cfg.password[i]));
        }
        wifi_buffer.push_back(0);

        Log::debug << "Storing SSID: '" << wifi_cfg.ssid
                   << "', size=" << wifi_cfg.ssid.size() << "\n";

        if (lfs_file_write(&piccante::fs::lfs, &writeFile, wifi_buffer.data(),
                           wifi_buffer.size()) > 0) {
            success = true;
        } else {
            Log::error << "Failed to write wifi settings file\n";
        }

        if (lfs_file_close(&piccante::fs::lfs, &writeFile) != LFS_ERR_OK) {
            Log::error << "Failed to close wifi settings file\n";
            success = false;
        }
    } else {
        Log::error << "Failed to open wifi settings file for writing\n";
    }

#endif

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

#ifdef WIFI_ENABLED

uint8_t get_wifi_mode() { return cfg.wifi_mode; }
void set_wifi_mode(uint8_t mode) { cfg.wifi_mode = mode; }
const wifi_settings& get_wifi_settings() { return wifi_cfg; }
void set_wifi_ssid(const std::string& ssid) { wifi_cfg.ssid = ssid; }
void set_wifi_password(const std::string& password) { wifi_cfg.password = password; }
void set_wifi_channel(uint8_t channel) { wifi_cfg.channel = channel; }
#endif

} // namespace piccante::sys::settings