#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>

namespace luna::config {
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
}  // namespace luna::config

#endif  //LUNA_PERIPHERAL_DEF_H
