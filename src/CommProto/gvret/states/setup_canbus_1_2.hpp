#pragma once

#include <cstdint>
#include <algorithm>
#include <utility>
#include <ostream>
#include <cstdint>
#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include "../gvret.hpp"

namespace gvret::state {
class setup_canbus_1_2 : public fsm::state<uint8_t, Protocol, bool> {
    // TODO: remove, use can2040 settings, or put to CanBus.hpp or smth.
    struct can_settings {
        uint32_t speed = 0;
        bool enabled = false;
        bool listenOnly = false;
    };

        public:
    explicit setup_canbus_1_2() : fsm::state<uint8_t, Protocol, bool>(SETUP_CANBUS_1_2) {}
    Protocol enter() override {
        step = 0;
        return SETUP_CANBUS_1_2;
    }

    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        auto settings = can_settings{
            .speed = bus_speed,
            .enabled = false,
            .listenOnly = false,
        };

        switch (step) { // NOLINT
            case 0:
                buff_int = byte;
                break;
            case 1:
                buff_int |= byte << 8U;
                break;
            case 2:
                buff_int |= byte << 16U;
                break;
            case 3:
                buff_int |= byte << 24U;
                bus_speed = buff_int & SPEED_MASK;
                bus_speed = std::min<uint32_t>(bus_speed, MAX_BUS_SPEED);


                if (buff_int == 0) {
                    settings.enabled = false;
                } else {
                    if (buff_int & BUS_FLAG_MASK) {
                        settings.enabled = buff_int & ENABLED_MASK;
                        settings.listenOnly = buff_int & LISTEN_ONLY_MASK;
                    } else {
                        // if not using extended status mode then just default to enabling
                        // - this was old behavior
                        settings.enabled = true;
                    }
                }
                // TODO: apply can settings
                break;
            case 4:
                buff_int = byte;
                break;
            case 5:
                buff_int |= byte << 8;
                break;
            case 6:
                buff_int |= byte << 16;
                break;
            case 7:
                buff_int |= byte << 24;
                bus_speed = buff_int & SPEED_MASK;
                if (bus_speed > MAX_BUS_SPEED)
                    bus_speed = MAX_BUS_SPEED;

                if (buff_int == 0) { // TODO: Add check if second Canbus is there.
                    settings.enabled = false;
                } else {
                    if (buff_int & BUS_FLAG_MASK) {
                        settings.enabled = buff_int & ENABLED_MASK;
                        settings.listenOnly = buff_int & LISTEN_ONLY_MASK;
                    } else {
                        // if not using extended status mode then just default to enabling
                        // - this was old behavior
                        settings.enabled = true;
                    }
                }
                // TODO: apply can settings

                // TODO: Write settings to eeprom
                return {IDLE, true};
        }
        step++;
        return {SETUP_CANBUS_1_2, true};
    }

        protected:
    uint8_t step = 0;
    uint32_t buff_int = 0;
    uint32_t bus_speed = 0;
};
} // namespace gvret::state