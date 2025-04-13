
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
#include <vector>
#include <string_view>
#include <map>
#include <functional>
#include "outstream/stream.hpp"
#include "settings.hpp"
#include "CommProto/gvret/handler.hpp"

namespace piccante::sys::shell {


class handler {
        public:
    explicit handler(gvret::handler& gvret, out::stream& out_stream,
                     const settings::system_settings& cfg = settings::get())
        : gvret(gvret), host_out(out_stream), cfg(cfg) {};
    ~handler() = default;

    // Disable copy and move operations
    handler(const handler&) = delete;
    handler& operator=(const handler&) = delete;
    handler(handler&&) = delete;
    handler& operator=(handler&&) = delete;

    void process_byte(char byte);
    void handle_cmd(const std::string_view& cmd);

        private:
    gvret::handler& gvret;
    out::stream& host_out;
    std::vector<char> buffer{};
    const settings::system_settings& cfg;

    struct CommandInfo {
        std::string_view description;
        void (handler::*func)(const std::string_view&);
        void operator()(handler* self, const std::string_view& arg) const {
            (self->*func)(arg);
        }
    };
    static std::map<std::string_view, CommandInfo, std::less<>> commands;

    // Command handlers
    void cmd_echo(const std::string_view& arg);
    void cmd_help(const std::string_view& arg);
    void cmd_toggle_binary(const std::string_view& arg);
    void cmd_can_enable(const std::string_view& arg);
    void cmd_can_disable(const std::string_view& arg);
    void cmd_can_bitrate(const std::string_view& arg);
    void cmd_can_status(const std::string_view& arg);
    void cmd_can_num_busses(const std::string_view& arg);
    void cmd_settings_show(const std::string_view& arg);
    void cmd_settings_store(const std::string_view& arg);
    void cmd_led_mode(const std::string_view& arg);
    void cmd_log_level(const std::string_view& arg);
    void cmd_sys_stats(const std::string_view& arg);
};
} // namespace piccante::sys::shell