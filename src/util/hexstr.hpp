#pragma once
#include <cstddef>
#include <string_view>

namespace hex {

static unsigned int parse(char byte) {
    constexpr char base = 10;

    if (byte >= '0' && byte <= '9') {
        return byte - '0';
    }
    if (byte >= 'A' && byte <= 'F') {
        return base + byte - 'A';
    }
    if (byte >= 'a' && byte <= 'f') {
        return base + byte - 'a';
    }
    return 0;
}

static unsigned int parse(const std::string_view& str, size_t len = 0) {
    unsigned int res = 0;
    if (len == 0) {
        len = str.length();
    }
    for (size_t i = 0; i < len; i++) {
        res += parse(str[i]) << (sizeof(unsigned int) * (len - 1 - i));
    }
    return res;
}
} // namespace hex