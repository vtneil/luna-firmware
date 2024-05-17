#ifndef LUNA_FIRMWARE_LUNA_PIN_DEF_H
#define LUNA_FIRMWARE_LUNA_PIN_DEF_H

#include <PinNames.h>

namespace luna::pins {
    namespace gpio {
        constexpr PinName LED_R  = PE_12;  // Onboard RGB LED R
        constexpr PinName LED_G  = PE_13;  // Onboard RGB LED G
        constexpr PinName LED_B  = PE_14;  // Onboard RGB LED B
        constexpr PinName BUZZER = PD_9;   // Onboard buzzer
        constexpr PinName USER_1 = PF_8;   // Onboard push button 1
        constexpr PinName USER_2 = PF_9;   // Onboard push button 2
    }  // namespace gpio

    namespace pyro {
        constexpr PinName SIG_A  = PG_3;
        constexpr PinName SIG_B  = PG_4;
        constexpr PinName SIG_C  = PG_5;

        constexpr PinName SENS_A = PG_6;
        constexpr PinName SENS_B = PG_7;
        constexpr PinName SENS_C = PG_8;
    }  // namespace pyro

    namespace comm {
        namespace rfd900x {
            constexpr PinName CTS     = PB_14;
            constexpr PinName RTS     = PB_15;
            constexpr PinName UART_TX = PD_1;
            constexpr PinName UART_RX = PD_0;
        }  // namespace rfd900x

        namespace lora {
            constexpr PinName AUX     = PF_3;
            constexpr PinName M1      = PF_4;
            constexpr PinName M0      = PF_5;
            constexpr PinName RST     = PB_2;
            constexpr PinName UART_TX = PD_5;
            constexpr PinName UART_RX = PD_6;
        }  // namespace lora
    }  // namespace comm

    namespace spi {
        namespace ch1 {
            constexpr PinName SCK  = PA_5;
            constexpr PinName MISO = PA_6;
            constexpr PinName MOSI = PA_7;
        }  // namespace ch1

        namespace ch3 {
            constexpr PinName SCK  = PC_10;
            constexpr PinName MISO = PC_11;
            constexpr PinName MOSI = PC_12;
        }  // namespace ch3

        namespace ch4 {
            constexpr PinName SCK  = PE_2;
            constexpr PinName MISO = PE_5;
            constexpr PinName MOSI = PE_6;
        }  // namespace ch4

        namespace cs {
            constexpr PinName icm42688 = PG_10;
            constexpr PinName icm20948 = PE_3;
            constexpr PinName ms1      = PG_11;
            constexpr PinName ms2      = PE_4;
            constexpr PinName sd       = PC_1;
            constexpr PinName flash    = PD_10;
        }  // namespace cs
    }  // namespace spi

    namespace tmc2209 {
        constexpr PinName UART_TX = PC_6;
        constexpr PinName UART_RX = PC_7;
    }  // namespace tmc2209
}  // namespace luna::pins

constexpr unsigned long UART_BAUD = 115200;

#endif  //LUNA_FIRMWARE_LUNA_PIN_DEF_H
