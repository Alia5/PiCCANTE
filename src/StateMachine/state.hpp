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
#include <string>
#include <type_traits>
#include <utility>

namespace piccante::fsm {

template <typename T, typename ID = std::string, typename ReturnType = void> class state {
        public:
    explicit state(ID state_id = ID{}) : id(std::move(state_id)) {}
    state(const state&) = default;
    state& operator=(const state&) = default;
    state(state&&) noexcept = default;
    state& operator=(state&&) noexcept = default;
    virtual ~state() = default;

    using tick_return_type =
        std::conditional_t<std::is_void_v<ReturnType>, ID, std::pair<ID, ReturnType>>;


    virtual tick_return_type
    tick([[maybe_unused]] T& data) = 0; // Now returns IdType instead of string
    virtual ID enter() { return id; } // Called when entering the state
    virtual void exit() {}                      // Called when exiting the state

    const ID& get_id() const { return id; }

        protected:
    ID id; // Now using IdType instead of hardcoded std::string
};

} // namespace fsm