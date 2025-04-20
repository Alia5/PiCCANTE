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
#include "httpd.hpp"
extern "C" {
#include "server/httpserver.h"
}
#include <pico/cyw43_arch.h>
#include <map>
#include <string>
#include <array>

#include <lfs.h>
#include "fs/littlefs_driver.hpp"
#include "Logger/Logger.hpp"

#include <FreeRTOS.h>
#include <semphr.h>

#include "api/settings.hpp"


namespace piccante::httpd {

namespace {
http_server_instance server = nullptr;
http_zone root_zone;
http_zone api_zone;

const std::map<std::string, std::string> mime_map{
    {".html", "text/html"},
    {".css", "text/css"},
    {".js", "application/javascript"},
    {".json", "application/json"},
    {".png", "image/png"},
    {".jpg", "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".gif", "image/gif"},
    {".svg", "image/svg+xml"},
    {".ico", "image/x-icon"},
};
SemaphoreHandle_t lfs_mutex = nullptr;

bool file_handler(http_connection conn, enum http_request_type type, char* path,
                  void* /*context*/) {
    if (!lfs_mutex) {
        return false;
    }

    Log::debug << "File handler: " << path << "\n";

    xSemaphoreTake(lfs_mutex, portMAX_DELAY);

    std::string p_str = (path && path[0]) ? path : "/";
    if (p_str == "/" || p_str == "/index" || p_str == "") {
        p_str = "/index.html";
    }
    std::string web_path = std::string("web/") + p_str;
    lfs_file_t readFile;
    int err =
        lfs_file_open(&piccante::fs::lfs, &readFile, web_path.c_str(), LFS_O_RDONLY);

    if (err != LFS_ERR_OK) {
        // try gzip version
        auto p = web_path.append(".gz");
        err = lfs_file_open(&piccante::fs::lfs, &readFile, p.c_str(), LFS_O_RDONLY);
    }

    if (err != LFS_ERR_OK) {
        // still not found? if not includes . fall back to index.html
        if (p_str.find('.') == std::string::npos) {
            web_path = "web/index.html";
            err = lfs_file_open(&piccante::fs::lfs, &readFile, web_path.c_str(),
                                LFS_O_RDONLY);
            if (err != LFS_ERR_OK) {
                // try gzip version
                auto p = web_path.append(".gz");

                err =
                    lfs_file_open(&piccante::fs::lfs, &readFile, p.c_str(), LFS_O_RDONLY);
                if (err != LFS_ERR_OK) {
                    Log::error << "File not found: " << web_path << "\n";
                    xSemaphoreGive(lfs_mutex);
                    return false;
                }
            }
        } else {
            Log::error << "File not found: " << web_path << "\n";
            xSemaphoreGive(lfs_mutex);
            return false;
        }
    }

    Log::debug << "Found file at " << web_path << "\n";

    // extract mime-type from file extension
    std::string mime_type = "application/octet-stream";
    auto is_gzipped = false;
    std::string::size_type ext_pos = web_path.rfind('.');
    std::string::size_type gz_pos = web_path.rfind(".gz");

    if (gz_pos != std::string::npos && gz_pos == web_path.length() - 3) {
        is_gzipped = true;
        ext_pos = web_path.rfind('.', gz_pos - 1);
    }
    if (ext_pos != std::string::npos) {
        std::string ext = web_path.substr(
            ext_pos, gz_pos != std::string::npos ? gz_pos - ext_pos : std::string::npos);
        auto it = mime_map.find(ext);
        if (it != mime_map.end()) {
            mime_type = it->second;
        }
    }

    http_write_handle handle;
    if (is_gzipped) {
        Log::debug << "File is gzipped - writing headers...\n";
        handle = http_server_begin_write_reply(conn, "200 OK", mime_type.c_str(),
                                               "Content-Encoding: gzip\r\n");
    } else {
        handle = http_server_begin_write_reply(conn, "200 OK", mime_type.c_str(), "");
    }
    std::array<char, BUFFER_SIZE> buffer;
    lfs_ssize_t n{};
    while ((n = lfs_file_read(&piccante::fs::lfs, &readFile, buffer.data(),
                              buffer.size())) > 0) {
        Log::debug << "Writing file chunk of size " << n << "\n";
        http_server_write_raw(handle, buffer.data(), n);
    }
    lfs_file_close(&piccante::fs::lfs, &readFile);

    http_server_end_write_reply(handle, "");

    xSemaphoreGive(lfs_mutex);

    return true;
}

bool api_handler(http_connection conn, enum http_request_type type, char* path,
                 void* /*context*/) {
    std::string p_str = (path && path[0]) ? path : "/";

    if (p_str == "settings") {
        if (type == HTTP_GET) {
            return api::settings::get(conn, p_str);
        }

        if (type == HTTP_POST) {
            return api::settings::set(conn, p_str);
        }

        http_server_send_reply(conn, "405 Method Not Allowed", "text/plain",
                               "Method Not Allowed", -1);
        return true;
    }

    if (p_str == "save") {
        if (type == HTTP_POST) {
            return api::settings::save(conn, p_str);
        }

        http_server_send_reply(conn, "405 Method Not Allowed", "text/plain",
                               "Method Not Allowed", -1);
        return true;
    }

    return false;
}
} // namespace

void start() {
    if (!lfs_mutex) {
        lfs_mutex = xSemaphoreCreateMutex();
    }
    server = http_server_create(CYW43_HOST_NAME, CYW43_HOST_NAME, MAX_THREAD_COUNT,
                                BUFFER_SIZE);
    http_server_add_zone(server, &root_zone, "", file_handler, nullptr);
    http_server_add_zone(server, &api_zone, "/api", api_handler, nullptr);
}

} // namespace piccante::httpd