#pragma once
#include <string_view>

namespace hex {

static unsigned int parse(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
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