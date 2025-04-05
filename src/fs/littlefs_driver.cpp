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
constexpr uint32_t PROG_SIZE = 16;
constexpr uint32_t BLOCK_CYCLES = 500;

static SemaphoreHandle_t flash_mutex = nullptr; // NOLINT

inline uint32_t flash_addr(lfs_block_t block, lfs_off_t off) {
    auto const fs_start_addr = reinterpret_cast<uint32_t>(&__filesystem_start); // NOLINT

    // We know the section exists at the right place, but we need to compute
    // addresses based on the known size rather than __filesystem_end
    return (fs_start_addr - XIP_BASE) + (block * LFS_BLOCK_SIZE) + off;
}


void ensure_mutex_initialized() {
    if (flash_mutex == nullptr) {
        flash_mutex = xSemaphoreCreateMutex();
        configASSERT(flash_mutex);
    }
}

int flash_read(const struct lfs_config* /*c*/, lfs_block_t block, lfs_off_t off,
               void* buffer, lfs_size_t size) {
    uint32_t const xip_addr = flash_addr(block, off) + XIP_BASE;
    memcpy(buffer, reinterpret_cast<const void*>(xip_addr), size); // NOLINT
    return LFS_ERR_OK;
}

int flash_prog(const struct lfs_config* /*c*/, lfs_block_t block, lfs_off_t off,
               const void* buffer, lfs_size_t size) {
    ensure_mutex_initialized();

    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) != pdTRUE) {
        return LFS_ERR_IO;
    }

    uint32_t const addr = flash_addr(block, off);
    uint32_t const ints = save_and_disable_interrupts();
    uint32_t const aligned_addr = addr & ~(FLASH_PAGE_SIZE - 1);
    uint32_t const offset_in_page = addr - aligned_addr;

    if (offset_in_page != 0 || (size % FLASH_PAGE_SIZE) != 0) {
        std::array<uint8_t, FLASH_PAGE_SIZE> page_buffer;

        uint32_t const end_addr = addr + size;
        uint32_t curr_page = aligned_addr;

        while (curr_page < end_addr) {
            memcpy(page_buffer.data(),
                   reinterpret_cast<void*>(curr_page + XIP_BASE), // NOLINT
                   FLASH_PAGE_SIZE);

            uint32_t const start_offset = (addr > curr_page) ? (addr - curr_page) : 0;
            uint32_t const end_offset = (curr_page + FLASH_PAGE_SIZE > end_addr)
                                            ? (end_addr - curr_page)
                                            : FLASH_PAGE_SIZE;

            if (end_offset > start_offset) {
                uint32_t const buffer_offset = (curr_page + start_offset) - addr;
                if (buffer_offset < size) {
                    uint32_t copy_size = end_offset - start_offset;
                    if (buffer_offset + copy_size > size) {
                        copy_size = size - buffer_offset;
                    }

                    memcpy(page_buffer.data() + start_offset,
                           std::next(static_cast<const uint8_t*>(buffer), buffer_offset),
                           copy_size);
                }
            }

            flash_range_erase(curr_page, FLASH_SECTOR_SIZE);
            flash_range_program(curr_page, page_buffer.data(), FLASH_PAGE_SIZE);

            curr_page += FLASH_PAGE_SIZE;
        }
    } else {
        flash_range_program(addr, static_cast<const uint8_t*>(buffer), size);
    }

    restore_interrupts(ints);
    xSemaphoreGive(flash_mutex);

    return LFS_ERR_OK;
}

int flash_erase(const struct lfs_config* /*c*/, lfs_block_t block) {
    ensure_mutex_initialized();
    if (xSemaphoreTake(flash_mutex, portMAX_DELAY) != pdTRUE) {
        return LFS_ERR_IO;
    }

    uint32_t const addr = flash_addr(block, 0);
    uint32_t const ints = save_and_disable_interrupts();

    flash_range_erase(addr, LFS_BLOCK_SIZE);
    restore_interrupts(ints);

    xSemaphoreGive(flash_mutex);

    return LFS_ERR_OK;
}

// Sync operation (no-op for RP2040)
int flash_sync(const struct lfs_config* /*c*/) { return LFS_ERR_OK; }

lfs_config const cfg = {
    .read = flash_read,
    .prog = flash_prog,
    .erase = flash_erase,
    .sync = flash_sync,

    .read_size = READ_SIZE,
    .prog_size = PROG_SIZE,
    .block_size = LFS_BLOCK_SIZE,
    .block_count = LFS_BLOCK_COUNT,
    .block_cycles = BLOCK_CYCLES,
    .cache_size = LFS_CACHE_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,
};

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