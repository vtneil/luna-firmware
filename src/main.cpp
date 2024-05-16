/**
 * @file main.cpp
 * @author Vivatsathorn Thitasirivit
 * @date 13 May 2024
 * @brief LUNA Firmware
 */

#include <Arduino.h>
#include "Arduino_Extended.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include <SPI.h>
#include "luna_pin_def.h"
#include "smart_delay.h"
#include "tasks.h"
#include "message.h"
#include "fsm.h"

// Type aliases

using time_type   = uint32_t;
using smart_delay = vt::smart_delay<time_type>;
using task_type   = vt::task_t<vt::smart_delay, time_type>;

template<size_t N>
using dispatcher_type = vt::task_dispatcher<N, vt::smart_delay, time_type>;

// Hardware instances

// HardwareSerial Serial2(luna::pins::comm::lora::UART_RX, luna::pins::comm::lora::UART_TX);
// HardwareSerial Serial4(luna::pins::comm::rfd900x::UART_RX, luna::pins::comm::rfd900x::UART_TX);

// TwoWire

// Hardware references

// USBSerial &UART_USB          = Serial;
// HardwareSerial &UART_LORA    = Serial2;
// HardwareSerial &UART_RFD900X = Serial4;

// Software data

luna::header_t header;
luna::data_t data;

void setup() {
    // GPIO and Digital Pins
    gpio_config << luna::pins::gpio::LED_R
                << luna::pins::gpio::LED_G
                << luna::pins::gpio::LED_B
                << luna::pins::pyro::SIG_A
                << luna::pins::pyro::SIG_B
                << luna::pins::pyro::SIG_C
                << luna::pins::gpio::BUZZER;
    gpio_config >> luna::pins::gpio::USER_1 >> luna::pins::gpio::USER_2;

    // UART Interfaces
    // UART_USB.begin(UART_BAUD);
    // UART_LORA.begin(UART_BAUD);
    // UART_RFD900X.begin(UART_BAUD);
}

void loop() {
    gpio_write << Toggle(luna::pins::gpio::LED_R)
               << Toggle(luna::pins::gpio::LED_G)
               << Toggle(luna::pins::gpio::LED_B)
               << Toggle(luna::pins::pyro::SIG_A)
               << Toggle(luna::pins::pyro::SIG_B)
               << Toggle(luna::pins::gpio::BUZZER);

    digitalWriteFast(luna::pins::pyro::SIG_C, digitalReadFast(luna::pins::gpio::USER_1) || digitalReadFast(luna::pins::gpio::USER_2));

    // UART_USB << "Clock" << SystemCoreClock << stream::crlf;

    delayMicroseconds(250UL * 1000UL);
}
