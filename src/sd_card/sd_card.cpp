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
#include "sd_card.hpp"

#include <string_view>
#include <span>
#include "fmt.hpp"
#include "Logger/Logger.hpp"

#include "ff_headers.h"
#include "ff_sddisk.h"
#include "ff_stdio.h"
#include "ff_utils.h"
//
#include "hw_config.h"

namespace piccante::sd {

namespace {
FF_Disk_t* disk = nullptr;
}

bool mount() {
    disk = FF_SDDiskInit("sd0");
    if (!disk) {
        Log::error << "FF_SDDiskInit failed: disk is nullptr\n";
        return false;
    }
    FF_Error_t err = FF_SDDiskMount(disk);
    if (FF_isERR(err) != pdFALSE) {
        Log::error << "Failed to mount SD card: " << (const char*)FF_GetErrMessage(err)
                   << "\n";
        FF_SDDiskDelete(disk);
        disk = nullptr;
        return false;
    }
    FF_FS_Add("/sd0", disk);
    return true;
}
bool unmount() {
    if (disk == nullptr) {
        return false;
    }
    FF_FS_Remove("/sd0");
    FF_Error_t err = FF_SDDiskUnmount(disk);
    if (FF_isERR(err) != pdFALSE) {
        Log::error << "Failed to unmount SD card: " << (const char*)FF_GetErrMessage(err)
                   << "\n";
        return false;
    }
    FF_SDDiskDelete(disk);
    disk = nullptr;
    return true;
}

bool is_mounted() {
    if (disk == nullptr) {
        return false;
    }
    return true;
}

FF_FILE* open_file(const std::string_view& filename, const std::string_view& mode) {
    if (disk == nullptr) {
        return 0;
    }
    const auto path = fmt::sprintf("/sd0/%s", filename.data());
    FF_FILE* file = ff_fopen(path.c_str(), mode.data());
    if (file == nullptr) {
        Log::error << "ff_fopen failed: \"" << path << "\" " << strerror(stdioGET_ERRNO())
                   << " " << stdioGET_ERRNO() << "\n";
    }
    return file;
}
void close_file(FF_FILE* file) {
    if (file == nullptr || disk == nullptr) {
        return;
    }
    if (-1 == ff_fclose(file)) {
        Log::error << "ff_fclose failed: " << strerror(stdioGET_ERRNO()) << " "
                   << stdioGET_ERRNO() << "\n";
        return;
    }
}

bool read_file(FF_FILE* file, std::span<char, std::dynamic_extent> buffer) {
    if (file == nullptr || disk == nullptr) {
        return false;
    }

    if (FF_Read(file, sizeof(char), buffer.size(),
                reinterpret_cast<uint8_t*>(buffer.data())) < 0) {
        Log::error << "ff_fread failed: " << strerror(stdioGET_ERRNO()) << " "
                   << stdioGET_ERRNO() << "\n";
        return false;
    }

    return true;
}
bool write_file(FF_FILE* file, const std::span<char, std::dynamic_extent> data) {
    if (file == nullptr || disk == nullptr) {
        return false;
    }
    if (FF_Write(file, sizeof(char), data.size(),
                 reinterpret_cast<uint8_t*>(data.data())) < 0) {
        Log::error << "ff_fprintf failed: " << strerror(stdioGET_ERRNO()) << " "
                   << stdioGET_ERRNO() << "\n";
        return false;
    }
    return true;
}

bool write_file(FF_FILE* file, const std::string_view data) {
    if (file == nullptr || disk == nullptr) {
        return false;
    }
    uint8_t* buffer = reinterpret_cast<uint8_t*>(const_cast<char*>(data.data()));

    if (FF_Write(file, sizeof(char), data.size(), buffer) < 0) {
        Log::error << "ff_fprintf failed: " << strerror(stdioGET_ERRNO()) << " "
                   << stdioGET_ERRNO() << "\n";
        return false;
    }
    return true;
}


} // namespace piccante::sd

void my_assert_func(const char* file, int line, const char* func, const char* pred) {
    piccante::Log::error << pcTaskGetName(NULL) << ": assertion \"" << pred
                         << "\" failed: file \"" << file << "\", line " << line
                         << ", function: " << func << "\n";
    vTaskSuspendAll();
}