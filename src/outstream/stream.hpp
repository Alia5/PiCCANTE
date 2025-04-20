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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>
#include <functional>
#include <vector>
#include "fmt.hpp"

#ifdef write
#undef write
#endif

namespace piccante::out {


class base_sink {
        public:
    virtual void write(const char* v, std::size_t s) = 0;
    virtual void flush() = 0;

    virtual ~base_sink() = default;
};

class sink_mux : public base_sink {
        public:
    explicit sink_mux() = default;

    void add_sink(base_sink* sink) { sinks_.push_back(sink); }
    void remove_sink(base_sink* sink) {
        sinks_.erase(std::remove(sinks_.begin(), sinks_.end(), sink), sinks_.end());
    }

    void write(const char* v, std::size_t s) override {
        for (auto& sink : sinks_) {
            sink->write(v, s);
        }
    }

    void flush() override {
        for (auto& sink : sinks_) {
            sink->flush();
        }
    }

        private:
    std::vector<base_sink*> sinks_;
};

class stream {
        private:
    base_sink& sink_;

        public:
    explicit stream(base_sink& sink) : sink_(sink) {}

    stream& operator<<(const std::string_view& str) {
        sink_.write(str.data(), str.size());
        return *this;
    }
    stream& operator<<(const std::string& str) {
        sink_.write(str.c_str(), str.length());
        return *this;
    }

    stream& operator<<(const char* str) {
        if (str) {
            sink_.write(str, strlen(str));
        }
        return *this;
    }

    template <typename T>
        requires(sizeof(T) == 1)
    stream& operator<<(T value) {
        sink_.write(reinterpret_cast<const char*>(&value), 1);
        return *this;
    }


    template <typename T, size_t N> stream& operator<<(const T (&arr)[N]) {
        if constexpr (std::is_same_v<T, char>) {
            sink_.write(arr, strlen(arr));
        } else if constexpr (std::is_same_v<T, uint8_t> ||
                             std::is_same_v<T, unsigned char>) {
            sink_.write(reinterpret_cast<const char*>(arr), N);
        } else {
            *this << "[";
            for (size_t i = 0; i < N; ++i) {
                if (i > 0)
                    *this << ", ";
                *this << arr[i];
            }
            *this << "]";
        }
        return *this;
    }

    template <typename T>
        requires std::is_integral_v<T> && (sizeof(T) > 1)
    stream& operator<<(T value) {
        std::string result;
        if constexpr (std::is_unsigned_v<T>) {
            result = fmt::sprintf("%lu", static_cast<unsigned long>(value));
        } else {
            result = fmt::sprintf("%ld", static_cast<long>(value));
        }
        if (!result.empty()) {
            sink_.write(result.c_str(), result.length());
        }
        return *this;
    }

    template <typename T>
        requires std::is_floating_point_v<T>
    stream& operator<<(T value) {
        std::string result = fmt::sprintf("%.6f", static_cast<double>(value));
        sink_.write(result.c_str(), result.length());
        return *this;
    }

    template <typename T>
        requires(!std::is_integral_v<T> && !std::is_floating_point_v<T> &&
                 !std::is_same_v<T, bool> && !std::is_same_v<T, char> &&
                 !std::is_same_v<T, const char*> &&
                 !std::is_convertible_v<T, std::string_view>)
    stream& operator<<(T* ptr) {
        std::string result = fmt::sprintf("%p", static_cast<const void*>(ptr));
        sink_.write(result.c_str(), result.length());
        return *this;
    }

    template <typename T>
        requires(requires(const T& t) {
                    { t.data() } -> std::convertible_to<const typename T::value_type*>;
                    { t.size() } -> std::convertible_to<std::size_t>;
                }) &&
                (std::is_same_v<typename T::value_type, char> ||
                 std::is_same_v<typename T::value_type, unsigned char> ||
                 std::is_same_v<typename T::value_type, uint8_t>) &&
                (!std::is_convertible_v<T, std::string_view>) &&
                (!std::is_same_v<T, std::string>)
    stream& operator<<(const T& container) {
        // Direct raw output without formatting
        sink_.write(reinterpret_cast<const char*>(container.data()), container.size());
        return *this;
    }


    template <typename T>
        requires(sizeof(T) == 1)
    [[maybe_unused]] size_t write(T value) {
        sink_.write(&value, 1);
        return 1;
    }

    template <typename T> [[maybe_unused]] size_t write(const T* buffer, size_t size) {
        sink_.write(reinterpret_cast<const char*>(buffer), size);
        return size;
    }


    void flush() { sink_.flush(); }

    base_sink& sink() { return sink_; }
};

} // namespace piccante::out
