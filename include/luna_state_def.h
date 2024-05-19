#ifndef LUNA_STATE_DEF_H
#define LUNA_STATE_DEF_H

namespace luna {
    enum class state_t : uint8_t {
        STARTUP = 0,
    };

    enum class pyro_state_t : uint8_t {
        DISARMED = 0,
        ARMED,
        FIRED
    };
}  // namespace luna

#endif  //LUNA_STATE_DEF_H
