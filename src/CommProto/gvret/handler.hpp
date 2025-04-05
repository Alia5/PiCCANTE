#pragma once

#include <cstdint>
#include <ostream>

#include "../../StateMachine/StateMachine.hpp"
#include "proto.hpp"
#include "states/states.hpp"

namespace gvret {

class handler {
        public:
    explicit handler(std::ostream& out_stream) : host_out(out_stream) {};
    virtual ~handler() = default;

    // Disable copy and move operations
    handler(const handler&) = delete;
    handler& operator=(const handler&) = delete;
    handler(handler&&) = delete;
    handler& operator=(handler&&) = delete;

    bool process_byte(uint8_t byte);

        private:
    std::ostream& host_out;
    fsm::StateMachine<uint8_t, Protocol, bool> protocol_fsm{
        state::idle(),
        state::get_command(),
        state::build_can_frame(),
        state::time_sync(host_out),
        state::get_d_inputs(host_out),
        state::get_a_inputs(host_out),
        state::set_d_out(),
        state::setup_canbus_1_2(),
        state::get_canbus_params_1_2(host_out),
        state::get_dev_info(host_out),
        state::set_sw_mode(),
        state::keepalive(host_out),
        state::set_systype(),
        state::echo_can_frame(host_out),
        state::get_num_buses(host_out),
        state::get_canbus_params_3_4_5(host_out),
        state::set_canbus_params_3_4_5(),
        // TODO: impl:
        state::send_fd_frame(),
        state::setup_fd(),
        state::get_fd_settings(),
    };
};


} // namespace gvret