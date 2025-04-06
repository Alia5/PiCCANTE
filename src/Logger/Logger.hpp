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
#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <map>
#include <cstdint>
#include "outstream/stream.hpp"
#include "outstream/uart_stream.hpp"

namespace piccante {
class Log {
        public:
    enum Level : uint8_t {
        LEVEL_DEBUG = 0,
        LEVEL_INFO,
        LEVEL_WARNING,
        LEVEL_ERROR,
    };

    static void set_log_level(Level level);
    static void init(Level level = LEVEL_INFO,
                     out::stream& out_stream = piccante::uart::out0);

        private:
    static Level current_level;
    static std::reference_wrapper<out::stream> out;

    static std::map<Level, std::string> level_names;

    template <typename... Args>
    static void log(const Level level, const std::string_view& message,
                    const Args&... args) {
        if (current_level <= level) {
            auto& ostr = out.get();
            ostr << "[" << level_names[level] << "] " << message;
            if constexpr (sizeof...(args) > 0) {
                ostr << ' ';
                ((ostr << args << ' '), ...);
            }
        }
    }

    class LogSink : public out::base_sink {
            protected:
        Level level;
        bool at_line_start = true;

        void output_message(const char* value, std::size_t size) {
            if (current_level <= level) {
                auto& output = out.get();

                for (size_t i = 0; i < size; ++i) {
                    if (at_line_start) {
                        output << "[" << level_names[level] << "] ";
                        at_line_start = false;
                    }

                    output << value[i];

                    if (value[i] == '\n') {
                        at_line_start = true;
                    }
                }
            }
        }

            public:
        explicit LogSink(Level log_level) : level(log_level) {}

        void write(const char* value, std::size_t size) override {
            output_message(value, size);
        }

        void flush() override { out.get().flush(); }
    };

    inline static LogSink debug_sink = LogSink(LEVEL_DEBUG);
    inline static LogSink info_sink = LogSink(LEVEL_INFO);
    inline static LogSink warning_sink = LogSink(LEVEL_WARNING);
    inline static LogSink error_sink = LogSink(LEVEL_ERROR);

        public:
    inline static out::stream debug = out::stream(debug_sink);
    inline static out::stream info = out::stream(info_sink);
    inline static out::stream warning = out::stream(warning_sink);
    inline static out::stream error = out::stream(error_sink);
};
}