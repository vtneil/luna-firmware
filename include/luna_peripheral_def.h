#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>

namespace luna::config {
    constexpr uint32_t TIME_TO_APOGEE_MIN  = 25 * 1000ul;
    constexpr uint32_t TIME_TO_APOGEE_MAX  = 30 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MIN = 10 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MAX = 10 * 1000ul;

    namespace alg {
        constexpr uint32_t LAUNCH_TON  = 100ul;  // 100 ms
        constexpr uint32_t LAUNCH_TOFF = 50ul;   // 50 ms
        constexpr double LAUNCH_ACC    = 50.0;
        // constexpr uin
    }  // namespace alg

    constexpr unsigned long RFD900X_BAUD      = 460800;
    constexpr unsigned long UART_BAUD         = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT  = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ       = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE      = 512ul;

    constexpr uint32_t TX_IDLE_INTERVAL       = 1000ul;  // 1 Hz
    constexpr uint32_t TX_ARMED_INTERVAL      = 500ul;   // 2 Hz
    constexpr uint32_t TX_PAD_PREOP_INTERVAL  = 200ul;   // 5 Hz
    constexpr uint32_t TX_ASCEND_INTERVAL     = 100ul;   // 10 Hz
    constexpr uint32_t TX_DESCEND_INTERVAL    = 200ul;   // 5 Hz

    constexpr uint32_t LOG_IDLE_INTERVAL      = 1000ul;  // 1 Hz
    constexpr uint32_t LOG_ARMED_INTERVAL     = 500ul;   // 2 Hz
    constexpr uint32_t LOG_PAD_PREOP_INTERVAL = 100ul;   // 10 Hz
    constexpr uint32_t LOG_ASCEND_INTERVAL    = 50ul;    // 20 Hz
    constexpr uint32_t LOG_DESCEND_INTERVAL   = 100ul;   // 10 Hz

    namespace details::assertions {
        static_assert(TIME_TO_APOGEE_MAX >= TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
        static_assert(TIME_TO_BURNOUT_MAX >= TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
    }  // namespace details::assertions
}  // namespace luna::config


#endif  //LUNA_PERIPHERAL_DEF_H
