#pragma once
#include <string>
#include <type_traits>
#include <utility>

namespace fsm {

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