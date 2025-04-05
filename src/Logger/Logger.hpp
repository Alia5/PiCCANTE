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
#include <iostream>
#include <map>
#include <cstdint>
#include "../outstream/outstream.hpp"

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
    static void init(Level level = LEVEL_INFO, std::ostream& out_stream = std::cout,
                     std::ostream& err_stream = std::cerr);

        private:
    static Level current_level;
    static std::reference_wrapper<std::ostream> out;
    static std::reference_wrapper<std::ostream> err;

    static std::map<Level, std::string> level_names;

    template <typename... Args>
    static void log(const Level level, const std::string_view& message,
                    const Args&... args) {
        if (current_level <= level) {
            auto ostr = (level == LEVEL_ERROR) ? err : out;
            ostr.get() << "[" << level_names[level] << "] " << message;
            if constexpr (sizeof...(args) > 0) {
                ostr.get() << ' ';
                ((ostr.get() << args << ' '), ...);
            }
        }
    }

    class LogSink : public outstream::CustomSink {
            protected:
        Level level;
        bool at_line_start = true;

        void output_message(const char* value, std::size_t size) {
            if (current_level <= level) {
                auto& output = (level == LEVEL_ERROR) ? err.get() : out.get();

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

        void flush() override { (level == LEVEL_ERROR ? err.get() : out.get()).flush(); }
    };

    inline static LogSink debug_sink = LogSink(LEVEL_DEBUG);
    inline static LogSink info_sink = LogSink(LEVEL_INFO);
    inline static LogSink warning_sink = LogSink(LEVEL_WARNING);
    inline static LogSink error_sink = LogSink(LEVEL_ERROR);

        public:
    inline static outstream::Stream<char> debug = outstream::Stream<char>(debug_sink);
    inline static outstream::Stream<char> info = outstream::Stream<char>(info_sink);
    inline static outstream::Stream<char> warning = outstream::Stream<char>(warning_sink);
    inline static outstream::Stream<char> error = outstream::Stream<char>(error_sink);
};
}