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
#include "littlefs_driver.hpp"
#include <cstdint>
#include <cstring>
#include <array>
#include "FreeRTOS.h" // NOLINT
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <hardware/regs/addressmap.h>
#include <lfs.h>
#include "portmacrocommon.h"
#include "projdefs.h"
#include "semphr.h"
#include "Logger/Logger.hpp"

// These symbols are defined in the linker script
extern uint32_t const __filesystem_start; // NOLINT
extern uint32_t const __filesystem_end;   // NOLINT

namespace piccante::fs {

lfs_t lfs; // NOLINT

namespace {

constexpr uint32_t READ_SIZE = 16;
constexpr uint32_t PROG_SIZE = FLASH_PAGE_SIZE;
constexpr uint32_t BLOCK_CYCLES = 256;

static SemaphoreHandle_t flash_mutex = nullptr; // NOLINT

void ensure_mutex_initialized() {
    if (flash_mutex == nullptr) {
        flash_mutex = xSemaphoreCreateMutex();
        configASSERT(flash_mutex);
    }
}

int flash_read(const struct lfs_config* cfg, lfs_block_t block, lfs_off_t off,
               void* buffer, lfs_size_t size) {
    auto fs_mem = (uint8_t*)cfg->context;
    memcpy(buffer, &fs_mem[block * cfg->block_size + off], size);
    return LFS_ERR_OK;
}
int flash_prog(const struct lfs_config* cfg, lfs_block_t block, lfs_off_t off,
               const void* buffer, lfs_size_t size) {
    ensure_mutex_initialized();

    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) != pdTRUE) {
        return LFS_ERR_IO;
    }

    // Get XIP address and convert to flash address
    auto fs_mem = (uint8_t*)cfg->context;
    auto addr = (uint32_t)(&fs_mem[block * cfg->block_size + off] - (uint8_t*)XIP_BASE);

    auto ints = save_and_disable_interrupts();
    flash_range_program(addr, (const uint8_t*)buffer, size);
    restore_interrupts(ints);

    xSemaphoreGive(flash_mutex);
    return LFS_ERR_OK;
}

int flash_erase(const struct lfs_config* cfg, lfs_block_t block) {
    ensure_mutex_initialized();

    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) != pdTRUE) {
        return LFS_ERR_IO;
    }

    auto fs_mem = (uint8_t*)cfg->context;
    auto addr = (uint32_t)(&fs_mem[block * cfg->block_size] - (uint8_t*)XIP_BASE);

    auto ints = save_and_disable_interrupts();
    flash_range_erase(addr, cfg->block_size);
    restore_interrupts(ints);

    xSemaphoreGive(flash_mutex);
    return LFS_ERR_OK;
}


// Sync operation (no-op for RP2040)
int flash_sync(const struct lfs_config* /*c*/) { return LFS_ERR_OK; }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
lfs_config const cfg = {
    .context = (void*)&__filesystem_start, // NOLINT
    .read = flash_read,
    .prog = flash_prog,
    .erase = flash_erase,
    .sync = flash_sync,

    .read_size = READ_SIZE,
    .prog_size = PROG_SIZE,
    .block_size = FLASH_SECTOR_SIZE,
    .block_count = LFS_BLOCK_COUNT,
    .block_cycles = BLOCK_CYCLES,
    .cache_size = PROG_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,
};
#pragma GCC diagnostic pop


} // namespace

bool init() {
    ensure_mutex_initialized();

    int err = lfs_mount(&lfs, &cfg);
    if (err != LFS_ERR_OK) {
        Log::error << "Failed to mount LittleFS: " << err << "Formatting..." << "\n";
        err = lfs_format(&lfs, &cfg);
        if (err != LFS_ERR_OK) {
            Log::error << "Failed to format LittleFS: " << err << "\n";
            return false;
        }
        Log::debug << "LittleFS formatted successfully\n";
        err = lfs_mount(&lfs, &cfg);
        if (err != LFS_ERR_OK) {
            Log::error << "Failed to mount LittleFS after formatting: " << err << "\n";
            return false;
        }
    }

    return true;
}

} // namespace piccante::fs