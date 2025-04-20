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
    auto obj_pos = json.find("\"" + std::string(key) + "\"");
    if (obj_pos == std::string_view::npos) {
        return std::nullopt;
    }

    auto obj_start = json.find('{', obj_pos);
    if (obj_start == std::string_view::npos) {
        return std::nullopt;
    }

    int brace_count = 1;
    auto obj_end = obj_start + 1;
    while (obj_end < json.size() && brace_count > 0) {
        // Skip whitespace for better handling of formatted JSON
        if (json[obj_end] == ' ' || json[obj_end] == '\t' || json[obj_end] == '\n' ||
            json[obj_end] == '\r') {
            ++obj_end;
            continue;
        }

        if (json[obj_end] == '{') {
            ++brace_count;
        } else if (json[obj_end] == '}') {
            --brace_count;
        }
        ++obj_end;
    }

    if (brace_count == 0 && obj_end > obj_start + 1) {
        return json.substr(obj_start + 1, obj_end - obj_start - 2);
    }

    return std::nullopt;
}

} // namespace piccante::util::json