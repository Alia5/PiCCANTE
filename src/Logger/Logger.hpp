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

    // format with standart printf syntax
    template <typename... Args>
    static std::string fmt(const std::string_view& fmtstr, const Args&... args) {
        std::string result;
        size_t size =
            std::snprintf(nullptr, 0, fmtstr.data(), args...) + 1; // Extra space for '\0'
        if (size <= 0) {
            return "";
            // throw std::runtime_error("Error during formatting.");
        }
        result.resize(size);
        std::sprintf(result.data(), fmtstr.data(), args...);
        return result;
    }

        private:
    static Level current_level;

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