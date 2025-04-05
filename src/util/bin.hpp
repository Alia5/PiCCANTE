#pragma once

#include <ostream>

namespace piccante {
template <typename T> class bin {
        private:
    const T& i;

        public:
    explicit bin(const T& i) : i(i) {}
    friend std::ostream& operator<<(std::ostream& os, const bin& b) {
        os.write(reinterpret_cast<const char*>(&(b.i)), sizeof(T));
        return os;
    }
    const T& val() const { return i; }
};

template <typename T> class bin_be {
        private:
    const T& i;

        public:
    explicit bin_be(const T& i) : i(i) {}
    friend std::ostream& operator<<(std::ostream& os, const bin_be& b) {
        // Write bytes in reverse order
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&(b.i));
        for (size_t j = sizeof(T); j > 0; --j) {
            os.put(bytes[j - 1]);
        }
        return os;
    }
    const T& val() const { return i; }
};
} // namespace piccante