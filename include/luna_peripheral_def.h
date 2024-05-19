#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>

namespace luna::config {
    constexpr unsigned long RFD900X_BAUD     = 460800;
    constexpr unsigned long UART_BAUD        = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ      = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE     = 512ul;
}  // namespace luna::config

#endif  //LUNA_PERIPHERAL_DEF_H
