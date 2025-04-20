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
#pragma once

#include <string_view>
#include <optional>
#include <string>

namespace piccante::util::json {

inline std::optional<std::string_view> get_value(std::string_view json,
                                                 std::string_view key) {
    auto pos = json.find("\"" + std::string(key) + "\"");
    if (pos == std::string_view::npos) {
        return std::nullopt;
    }
    pos = json.find(':', pos);
    if (pos == std::string_view::npos) {
        return std::nullopt;
    }
    pos++;

    // Skip whitespace including newlines
    while (pos < json.size() &&
           (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' ||
            json[pos] == '\r' || json[pos] == '\"')) {
        ++pos;
    }

    auto end = pos;
    bool in_quotes = (pos > 0 && json[pos - 1] == '\"');
    if (in_quotes) {
        end = json.find('\"', pos);
    } else {
        // For non-quoted values, stop at comma, brace, or whitespace
        while (end < json.size() && json[end] != ',' && json[end] != '}' &&
               json[end] != '\n' && json[end] != '\r' && json[end] != ' ' &&
               json[end] != '\t') {
            ++end;
        }
    }
    if (end == std::string_view::npos) {
        return std::nullopt;
    }
    return json.substr(pos, end - pos);
}

inline std::optional<std::string_view> get_object(std::string_view json,
                                                  std::string_view key) {
    auto key_pos = json.find("\"" + std::string(key) + "\"");
    if (key_pos == std::string_view::npos) {
        return std::nullopt;
    }

    auto colon_pos = json.find(':', key_pos);
    if (colon_pos == std::string_view::npos) {
        return std::nullopt;
    }

    auto pos = colon_pos + 1;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t' ||
                                 json[pos] == '\n' || json[pos] == '\r')) {
        ++pos;
    }

    if (pos >= json.size() || json[pos] != '{') {
        return std::nullopt;
    }

    auto obj_start = pos;
    int brace_count = 1;
    auto obj_end = obj_start + 1;

    while (obj_end < json.size() && brace_count > 0) {
        if (json[obj_end] == '{') {
            ++brace_count;
        } else if (json[obj_end] == '}') {
            --brace_count;
        }
        ++obj_end;
    }

    if (brace_count == 0) {
        size_t content_start = obj_start;
        size_t content_length = (obj_end)-content_start;
        return json.substr(content_start, content_length);
    }

    return std::nullopt;
}

} // namespace piccante::util::json