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
#include "about.hpp"
#include <string_view>
#include <string>
extern "C" {
#include "../server/httpserver.h"
}
#include "fmt.hpp"

struct _http_connection;
typedef _http_connection* http_connection;
typedef _http_connection* http_write_handle;

namespace piccante::httpd::api::about {
bool get(http_connection conn, [[maybe_unused]] std::string_view url) {
    const auto version_string = fmt::sprintf(R"({"version":"%s"})", PICCANTE_VERSION);
    http_server_send_reply(conn, "200 OK", "application/json", version_string.c_str(),
                           version_string.size());
    return true;
}
} // namespace piccante::httpd::api::about