#ifndef LUNA_FIRMWARE_LUNA_PIN_DEF_H
#define LUNA_FIRMWARE_LUNA_PIN_DEF_H

#include <PinNames.h>

namespace luna {
    namespace pyro {
        constexpr PinName SIG_A  = PinName::PG_3;
        constexpr PinName SIG_B  = PinName::PG_4;
        constexpr PinName SIG_C  = PinName::PG_5;

        constexpr PinName SENS_A = PinName::PG_6;
        constexpr PinName SENS_B = PinName::PG_7;
        constexpr PinName SENS_C = PinName::PG_8;
    }  // namespace pyro

    namespace comm {
        namespace rfd900x {
            constexpr PinName CTS     = PinName::PB_14;
            constexpr PinName RTS     = PinName::PB_15;
            constexpr PinName UART_TX = PinName::PD_1;
            constexpr PinName UART_RX = PinName::PD_0;
        }  // namespace rfd900x

        namespace lora {
            constexpr PinName AUX     = PinName::PF_3;
            constexpr PinName M1      = PinName::PF_4;
            constexpr PinName M0      = PinName::PF_5;
            constexpr PinName RST     = PinName::PB_2;
            constexpr PinName UART_TX = PinName::PD_5;
            constexpr PinName UART_RX = PinName::PD_6;
        }  // namespace lora
    }  // namespace comm
}  // namespace luna

constexpr unsigned long UART_BAUD = 115200;

#endif  //LUNA_FIRMWARE_LUNA_PIN_DEF_H
