#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>
#include "luna_pin_def.h"

namespace luna::config {
    constexpr uint32_t PYRO_ACTIVATE_INTERVAL = 1000ul;

    constexpr auto HZ_TO_INTERVAL_MS          = [](const double FREQUENCY_HZ) -> uint32_t {
        return static_cast<uint32_t>(1000. / FREQUENCY_HZ);
    };

    constexpr auto INTERVAL_MS_TO_HZ = [](const uint32_t INTERVAL) -> uint32_t {
        return 1000 / INTERVAL;
    };

    constexpr uint8_t INA236_ADDRESS = 0x40;

    enum INA236ADCRange {
        RANGE_80MV = 0,
        RANGE_20MV = 1
    };

    constexpr uint32_t TIME_TO_APOGEE_MIN  = 23 * 1000ul;
    constexpr uint32_t TIME_TO_APOGEE_MAX  = 27 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MIN = 4 * 1000ul;
    constexpr uint32_t TIME_TO_BURNOUT_MAX = 6 * 1000ul;

    namespace alg {
        constexpr uint32_t LAUNCH_TON          = 150ul;          // 150 ms
        constexpr uint32_t BURNOUT_TON         = 500ul;          // 500 ms
        constexpr uint32_t APOGEE_SLOW_TON     = 1000ul;         // 2000 ms
        constexpr uint32_t MAIN_DEPLOYMENT_TON = 200ul;          // 200 ms
        constexpr uint32_t LANDING_TON         = 5000ul;         // 200 ms
        constexpr double LAUNCH_ACC            = 40.0;           // 40 m/s^2
        constexpr double APOGEE_VEL            = 10.0;           // m/s
        constexpr double MAIN_ALTITUDE         = 450.f + 100.f;  // m
    }  // namespace alg

    constexpr unsigned long RFD900X_BAUD         = 460800;
    constexpr unsigned long UART_BAUD            = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT     = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ          = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE         = 512ul;

    constexpr uint32_t TX_IDLE_INTERVAL          = HZ_TO_INTERVAL_MS(1);  // 1 Hz
    constexpr uint32_t TX_ARMED_INTERVAL         = HZ_TO_INTERVAL_MS(2);  // 2 Hz
    constexpr uint32_t TX_PAD_PREOP_INTERVAL     = HZ_TO_INTERVAL_MS(4);  // 4 Hz
    constexpr uint32_t TX_ASCEND_INTERVAL        = HZ_TO_INTERVAL_MS(5);  // 5 Hz
    constexpr uint32_t TX_DESCEND_INTERVAL       = HZ_TO_INTERVAL_MS(4);  // 4 Hz

    constexpr uint32_t LOG_IDLE_INTERVAL         = HZ_TO_INTERVAL_MS(1);   // 1 Hz
    constexpr uint32_t LOG_ARMED_INTERVAL        = HZ_TO_INTERVAL_MS(2);   // 2 Hz
    constexpr uint32_t LOG_PAD_PREOP_INTERVAL    = HZ_TO_INTERVAL_MS(10);  // 10 Hz
    constexpr uint32_t LOG_ASCEND_INTERVAL       = HZ_TO_INTERVAL_MS(20);  // 20 Hz
    constexpr uint32_t LOG_DESCEND_INTERVAL      = HZ_TO_INTERVAL_MS(10);  // 10 Hz

    constexpr uint32_t BUZZER_ON_INTERVAL        = 50ul;                    // 50 ms
    constexpr uint32_t BUZZER_IDLE_INTERVAL      = HZ_TO_INTERVAL_MS(1);    // 1 Hz
    constexpr uint32_t BUZZER_ARMED_INTERVAL     = HZ_TO_INTERVAL_MS(2);    // 2 Hz
    constexpr uint32_t BUZZER_PAD_PREOP_INTERVAL = HZ_TO_INTERVAL_MS(10);   // 10 Hz
    constexpr uint32_t BUZZER_ASCEND_INTERVAL    = HZ_TO_INTERVAL_MS(0.2);  // 0.2 Hz
    constexpr uint32_t BUZZER_DESCEND_INTERVAL   = HZ_TO_INTERVAL_MS(1);    // 1 Hz

    constexpr auto BUZZER_OFF_INTERVAL           = [](const uint32_t BUZZER_TOTAL_INTERVAL) -> uint32_t {
        return BUZZER_TOTAL_INTERVAL - BUZZER_ON_INTERVAL;
    };

    namespace details::assertions {
        static_assert(TIME_TO_APOGEE_MAX >= TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
        static_assert(TIME_TO_BURNOUT_MAX >= TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
    }  // namespace details::assertions
}  // namespace luna::config


#endif  //LUNA_PERIPHERAL_DEF_H
