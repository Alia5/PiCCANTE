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
#include "pico/flash.h"
#include "hardware/sync.h"
#include <hardware/regs/addressmap.h>
#include <hardware/structs/spi.h>
#include <lfs.h>
#include "projdefs.h"
#include "semphr.h"
#include "Logger/Logger.hpp"
#include "pico/multicore.h"
#include <hardware/dma.h>

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

struct FlashProgramParams {
    uint32_t addr;
    const uint8_t* buffer;
    size_t size;
};

struct FlashEraseParams {
    uint32_t addr;
    size_t size;
};

static void flash_program_wrapper(void* params) {
    auto* p = static_cast<FlashProgramParams*>(params);
    flash_range_program(p->addr, p->buffer, p->size);
}

static void flash_erase_wrapper(void* params) {
    auto* p = static_cast<FlashEraseParams*>(params);
    flash_range_erase(p->addr, p->size);
}

int flash_prog(const struct lfs_config* cfg, lfs_block_t block, lfs_off_t off,
               const void* buffer, lfs_size_t size) {
    // Get XIP address and convert to flash address
    auto fs_mem = (uint8_t*)cfg->context;
    auto addr = (uint32_t)(&fs_mem[block * cfg->block_size + off] - (uint8_t*)XIP_BASE);

    FlashProgramParams params{addr, static_cast<const uint8_t*>(buffer), size};

    // Execute the flash operation safely
    int result = flash_safe_execute(flash_program_wrapper, &params, 100);
    if (result != PICO_OK) {
        Log::error << "Flash program failed with code: " << result << "\n";
        return LFS_ERR_IO;
    }


    return LFS_ERR_OK;
}


int flash_erase(const struct lfs_config* cfg, lfs_block_t block) {
    auto fs_mem = (uint8_t*)cfg->context;


    auto addr = (uint32_t)(&fs_mem[block * cfg->block_size] - (uint8_t*)XIP_BASE);


    // Set up the parameters for the flash operation
    FlashEraseParams params{addr, cfg->block_size};

    // Execute the flash operation safely
    int result = flash_safe_execute(flash_erase_wrapper, &params, 100);
    if (result != PICO_OK) {
        Log::error << "Flash erase failed with code: " << result << "\n";
        return LFS_ERR_IO;
    }


    return LFS_ERR_OK;
}

// Sync operation (no-op for RP2040)
int flash_sync(const struct lfs_config* /*c*/) { return LFS_ERR_OK; }

int lock(const struct lfs_config* /*c*/) {
    if (xSemaphoreTake(flash_mutex, 200) != pdTRUE) {
        Log::error << "Failed to take flash mutex\n";
        return LFS_ERR_IO;
    }
    // interrupts = save_and_disable_interrupts();
    return LFS_ERR_OK;
}
int unlock(const struct lfs_config* /*c*/) {
    // restore_interrupts(interrupts);
    xSemaphoreGive(flash_mutex);
    return LFS_ERR_OK;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
lfs_config const cfg = {
    .context = (void*)&__filesystem_start, // NOLINT
    .read = flash_read,
    .prog = flash_prog,
    .erase = flash_erase,
    .sync = flash_sync,
    .lock = lock,
    .unlock = unlock,
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