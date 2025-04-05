
#pragma once

#include <ostream>
#include <string>
#include <memory>
#include <map>
#include <type_traits>
#include "state.hpp"

#include "../Logger/Logger.hpp"

namespace fsm {

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
                        Log::error << "Transition to non-existent state\n" << std::flush;
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
                    Log::error << "Transition to non-existent state\n" << std::flush;
                }
            }
        } else {
            Log::error << "State not found\n" << std::flush;
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