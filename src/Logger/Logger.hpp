#pragma once
#include <string_view>
#include <iostream>
#include <map>
class Log {
        public:
    enum Level {
        LEVEL_DEBUG = 0,
        LEVEL_INFO,
        LEVEL_WARNING,
        LEVEL_ERROR,
    };

    static void set_log_level(Level level);
    static void init(Level level = LEVEL_INFO, std::ostream& out_stream = std::cout,
                     std::ostream& err_stream = std::cerr);

    template <class... Args>
    static void Debug(const std::string_view& message, const Args&... args) {
        log(LEVEL_DEBUG, message, args...);
    }

    template <class... Args>
    static void Info(const std::string_view& message, const Args&... args) {
        log(LEVEL_INFO, message, args...);
    }

    template <class... Args>
    static void Warning(const std::string_view& message, const Args&... args) {
        log(LEVEL_WARNING, message, args...);
    }
    template <class... Args>
    static void Error(const std::string_view& message, const Args&... args) {
        log(LEVEL_ERROR, message, args...);
    }

        private:
    static Level current_level;
    static std::reference_wrapper<std::ostream> out;
    static std::reference_wrapper<std::ostream> err;

    static std::map<Level, std::string> level_names;

    template <typename... Args>
    static void log(const Level level, const std::string_view& message,
                    const Args&... args) {
        if (current_level <= level) {
            std::cout << "[" << level_names[level] << "] " << message;
            if constexpr (sizeof...(args) > 0) {
                std::cout << ' ';
                ((std::cout << args << ' '), ...);
            }
            std::cout << std::endl;
        }
    }
};