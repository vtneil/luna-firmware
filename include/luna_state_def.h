#ifndef LUNA_STATE_DEF_H
#define LUNA_STATE_DEF_H

namespace luna {
    enum class state_t : uint8_t {
        STARTUP = 0,
        IDLE_SAFE,
        ARMED,
        PAD_PREOP,
        POWERED,
        COASTING,
        DROGUE_DEPLOY,
        DROGUE_DESCEND,
        MAIN_DEPLOY,
        MAIN_DESCEND,
        LANDED,
        RECOVERED_SAFE
    };

    inline const char *state_string(const state_t state) {
        switch (state) {
            case state_t::STARTUP:
                return "STARTUP";
            case state_t::IDLE_SAFE:
                return "IDLE_SAFE";
            case state_t::ARMED:
                return "ARMED";
            case state_t::POWERED:
                return "POWERED";
            case state_t::COASTING:
                return "COASTING";
            case state_t::DROGUE_DEPLOY:
                return "DROGUE_DEPLOY";
            case state_t::DROGUE_DESCEND:
                return "DROGUE_DESCEND";
            case state_t::MAIN_DEPLOY:
                return "MAIN_DEPLOY";
            case state_t::MAIN_DESCEND:
                return "MAIN_DESCEND";
            case state_t::LANDED:
                return "LANDED";
            case state_t::RECOVERED_SAFE:
                return "RECOVERED_SAFE";
            default:
                __builtin_unreachable();
        }
    }

    enum class pyro_state_t : uint8_t {
        DISARMED = 0,
        ARMED,
        FIRED
    };
}  // namespace luna

#endif  //LUNA_STATE_DEF_H
