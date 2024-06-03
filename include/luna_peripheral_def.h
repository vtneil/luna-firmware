#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>

namespace luna::config {
    constexpr uint32_t TIME_TO_APOGEE_MIN  = 25 * 1000ul;
    constexpr uint32_t TIME_TO_APOGEE_MAX  = 30 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MIN = 4 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MAX = 6 * 1000ul;

    namespace alg {
        constexpr uint32_t LAUNCH_TON  = 100ul;   // 100 ms
        constexpr uint32_t LAUNCH_TOFF = 50ul;    // 50 ms
        constexpr double LAUNCH_ACC    = 50.0;    // m/s^2
        constexpr double APOGEE_VEL    = 10.0;    // m/s
        constexpr float MAIN_ALTITUDE  = 600.0f;  // m
    }  // namespace alg

    constexpr unsigned long RFD900X_BAUD         = 460800;
    constexpr unsigned long UART_BAUD            = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT     = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ          = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE         = 512ul;

    constexpr uint32_t TX_IDLE_INTERVAL          = 1000ul;  // 1 Hz
    constexpr uint32_t TX_ARMED_INTERVAL         = 500ul;   // 2 Hz
    constexpr uint32_t TX_PAD_PREOP_INTERVAL     = 200ul;   // 5 Hz
    constexpr uint32_t TX_ASCEND_INTERVAL        = 200ul;   // 5 Hz
    constexpr uint32_t TX_DESCEND_INTERVAL       = 200ul;   // 5 Hz

    constexpr uint32_t LOG_IDLE_INTERVAL         = 1000ul;  // 1 Hz
    constexpr uint32_t LOG_ARMED_INTERVAL        = 500ul;   // 2 Hz
    constexpr uint32_t LOG_PAD_PREOP_INTERVAL    = 100ul;   // 10 Hz
    constexpr uint32_t LOG_ASCEND_INTERVAL       = 50ul;    // 20 Hz
    constexpr uint32_t LOG_DESCEND_INTERVAL      = 100ul;   // 10 Hz

    constexpr uint32_t BUZZER_ON_INTERVAL        = 25ul;    // 50 ms
    constexpr uint32_t BUZZER_IDLE_INTERVAL      = 1000ul;  // 1 Hz
    constexpr uint32_t BUZZER_ARMED_INTERVAL     = 500ul;   // 2 Hz
    constexpr uint32_t BUZZER_PAD_PREOP_INTERVAL = 100ul;   // 10 Hz
    constexpr uint32_t BUZZER_ASCEND_INTERVAL    = 5000ul;  // 0.2 Hz
    constexpr uint32_t BUZZER_DESCEND_INTERVAL   = 1000ul;  // 1 Hz

    constexpr auto BUZZER_OFF_INTERVAL           = [](const uint32_t BUZZER_TOTAL_INTERVAL) -> uint32_t {
        return BUZZER_TOTAL_INTERVAL - BUZZER_ON_INTERVAL;
    };

    namespace details::assertions {
        static_assert(TIME_TO_APOGEE_MAX >= TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
        static_assert(TIME_TO_BURNOUT_MAX >= TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
    }  // namespace details::assertions
}  // namespace luna::config


#endif  //LUNA_PERIPHERAL_DEF_H
