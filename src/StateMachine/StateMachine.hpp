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
#include <memory>
#include <map>
#include <type_traits>
#include "state.hpp"

#include "../Logger/Logger.hpp"

namespace piccante::fsm {

template <typename T, typename ID = std::string, typename ReturnType = void>
class StateMachine {
        public:
    using state_type = state<T, ID, ReturnType>;
    using tick_return_type = typename state_type::tick_return_type;

    StateMachine(const StateMachine&) = delete;
    StateMachine& operator=(const StateMachine&) = delete;

    // Default move constructor and assignment operator
    StateMachine(StateMachine&&) = default;
    StateMachine& operator=(StateMachine&&) = default;


    template <typename InitialState, typename... States>
    explicit StateMachine(InitialState&& initial, States&&... states) {
        // Get the ID before moving the state
        ID initial_id = initial.get_id();

        add_state(std::make_unique<std::decay_t<InitialState>>(
            std::forward<InitialState>(initial)));
        (add_state(std::make_unique<std::decay_t<States>>(std::forward<States>(states))),
         ...);
        current = initial_id;
    }

    virtual ~StateMachine() = default;

    // NOLINTNEXTLINE
    std::conditional_t<std::is_void_v<ReturnType>, void, ReturnType> tick(T& data) {
        auto it = states.find(current);
        if (it != states.end()) {
            ID next;
            if constexpr (std::is_void_v<ReturnType>) {
                auto result = it->second->tick(data);
                next = result;
            } else {
                auto result = it->second->tick(data);

                next = result.first;
                ReturnType return_value = result.second;


                while (next != current) {
                    it->second->exit();
                    current = next;
                    auto next_it = states.find(current);
                    if (next_it != states.end()) {
                        next = next_it->second->enter();
                    } else {
                        Log::error << "Transition to non-existent state\n";
                    }
                }

                return return_value;
            }

            while (next != current) {
                it->second->exit();
                current = next;
                auto next_it = states.find(current);
                if (next_it != states.end()) {
                    next = next_it->second->enter();
                } else {
                    Log::error << "Transition to non-existent state\n";
                }
            }
        } else {
            Log::error << "State not found\n";
            if constexpr (!std::is_void_v<ReturnType>) {
                return ReturnType{};
            }
        }
    }

        private:
    // NOLINTNEXTLINE
    template <typename StatePtr> void add_state(StatePtr&& state_ptr) {
        if (state_ptr) {
            ID id = state_ptr->get_id();
            // NOLINTNEXTLINE
            states.emplace(std::move(id), std::move(state_ptr));
        }
    }

    std::map<ID, std::unique_ptr<state<T, ID, ReturnType>>> states = {};
    ID current = ID{};
};

} // namespace fsm