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
        const auto* bytes = reinterpret_cast<const unsigned char*>(&(b.i));
        for (size_t j = sizeof(T); j > 0; --j) {
            os.put(bytes[j - 1]);
        }
        return os;
    }
    const T& val() const { return i; }
};
} // namespace piccante