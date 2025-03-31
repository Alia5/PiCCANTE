#pragma once

#include <string_view>
#include <cstdio>

namespace fmt
{

template <typename... Args>
static std::string sprintf(const std::string_view& fmtstr, const Args&... args) {
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

}