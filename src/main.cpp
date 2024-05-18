/**
 * @file main.cpp
 * @author Vivatsathorn Thitasirivit
 * @date 13 May 2024
 * @brief LUNA Firmware
 */

#include <Arduino.h>
#include "Arduino_Extended.h"
#include <STM32LowPower.h>
#include "File_Utility.h"

#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <SdFat.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <ICM_20948.h>
#include <ICM42688.h>
#include <MS5611_SPI.h>

#include "luna_config.h"
#include "luna_pin_def.h"
#include "luna_state_def.h"
#include "luna_peripheral_def.h"
#include "avionics_algorithm.h"
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

// Hardware interfaces

HardwareSerial Serial2(luna::pins::comm::lora::UART_RX, luna::pins::comm::lora::UART_TX);
HardwareSerial Serial4(luna::pins::comm::rfd900x::UART_RX, luna::pins::comm::rfd900x::UART_TX);

TwoWire Wire1(to_digital(luna::pins::i2c::ch1::SDA),
              to_digital(luna::pins::i2c::ch1::SCL));

SPIClass SPI_1(to_digital(luna::pins::spi::ch1::MOSI),
               to_digital(luna::pins::spi::ch1::MISO),
               to_digital(luna::pins::spi::ch1::SCK));
SPIClass SPI_3(to_digital(luna::pins::spi::ch3::MOSI),
               to_digital(luna::pins::spi::ch3::MISO),
               to_digital(luna::pins::spi::ch3::SCK));
SPIClass SPI_4(to_digital(luna::pins::spi::ch4::MOSI),
               to_digital(luna::pins::spi::ch4::MISO),
               to_digital(luna::pins::spi::ch4::SCK));

// Hardware references

// USBSerial &UART_USB          = Serial;
HardwareSerial &UART_LORA    = Serial2;
HardwareSerial &UART_RFD900X = Serial4;

// Peripherals

struct peripherals {
    uint8_t gnss_m9v;
    uint8_t imu_icm20948;
    uint8_t imu_icm42688;
    uint8_t ms1;
    uint8_t ms2;
    uint8_t flash;
    uint8_t sd;

    template<traits::has_ostream OStream>
    OStream &operator>>(OStream &ostream) {
        ostream << "GNSS M9V: " << gnss_m9v << stream::crlf;
        ostream << "IMU ICM20948: " << imu_icm20948 << stream::crlf;
        ostream << "IMU ICM42688: " << imu_icm42688 << stream::crlf;
        ostream << "BARO MS1: " << ms1 << stream::crlf;
        ostream << "BARO MS2: " << ms2 << stream::crlf;
        ostream << "STOR FLASH: " << flash << stream::crlf;
        ostream << "STOR SD: " << sd << stream::crlf;
        return ostream;
    }
} pvalid;

SdExFat flash;
SdExFat sd;

FsUtil<SdExFat, ExFile> flash_util(flash);
FsUtil<SdExFat, ExFile> sd_util(sd);

SFE_UBLOX_GNSS gnss_m9v;
ICM_20948_SPI imu_icm20948;
ICM42688 imu_icm42688(SPI_1, to_digital(luna::pins::spi::cs::icm42688));
MS5611_SPI ms1(to_digital(luna::pins::spi::cs::ms1), &SPI_1);
MS5611_SPI ms2(to_digital(luna::pins::spi::cs::ms2), &SPI_4);

// Software data

luna::sensor_data_t sensor_data;

// Communication data

luna::header_t header;
luna::data_t data;

// Software control
dispatcher_type<20> dispatcher;
vt::fsm<luna::state_t> fsm(luna::state_t::STARTUP);

extern void read_core();

extern void read_gnss();

extern void read_ms();

extern void read_icm20948();

extern void read_icm42688();

extern void accept_command();

extern void transmit_data();

extern void save_data();

extern void fsm_map();

void setup() {
    // GPIO and Digital Pins
    dout_low << luna::pins::gpio::LED_R
             << luna::pins::gpio::LED_G
             << luna::pins::gpio::LED_B
             << luna::pins::pyro::SIG_A
             << luna::pins::pyro::SIG_B
             << luna::pins::pyro::SIG_C
             << luna::pins::gpio::BUZZER;

    dout_high << luna::pins::spi::cs::flash
              << luna::pins::spi::cs::sd
              << luna::pins::spi::cs::ms2
              << luna::pins::spi::cs::ms1
              << luna::pins::spi::cs::icm20948
              << luna::pins::spi::cs::icm42688;

    din_config << luna::pins::pyro::SENS_A
               << luna::pins::pyro::SENS_B
               << luna::pins::pyro::SENS_C
               << luna::pins::gpio::USER_1
               << luna::pins::gpio::USER_2;

    // UART Interfaces
    // UART_USB.begin(UART_BAUD);
    UART_LORA.begin(UART_BAUD);
    UART_RFD900X.begin(UART_BAUD);

    // I2C (Wire) Interfaces
    Wire1.setClock(400000);
    Wire1.begin();

    // I2C Debug
    // i2c_detect(UART_RFD900X, Wire1, 0, 127);

    // SPI Interfaces
    // SPI 1, 2, 3: 64 MHz; SPI 4,5: 120 MHz

    // Storage Initialization
    pvalid.flash = flash.begin(SdSpiConfig(luna::pins::spi::cs::flash,
                                           SHARED_SPI,
                                           SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                                           &SPI_3));
    pvalid.sd    = sd.begin(SdSpiConfig(luna::pins::spi::cs::sd,
                                        SHARED_SPI,
                                        SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                                        &SPI_3));

    if (pvalid.flash) {
    }

    if (pvalid.sd) {
    }

    // Peripherals Initialization
    pvalid.ms1 = ms1.begin();
    pvalid.ms2 = ms2.begin();

    // //
    // if (const int status = imu_icm42688.begin(); status < 0) {
    //     UART_RFD900X.println("IMU initialization unsuccessful");
    //     UART_RFD900X.println("Check IMU wiring or try cycling power");
    //     UART_RFD900X.print("Status: ");
    //     UART_RFD900X.println(status);
    //     while (true)
    //         ;
    // }
    //
    // //
    // int valid = gnss_m9v.begin(Wire1);
    // if (valid) {
    //     gnss_m9v.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    //     gnss_m9v.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    //     gnss_m9v.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    //     gnss_m9v.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    //
    //     // Optional
    //     gnss_m9v.saveConfiguration(UBLOX_CUSTOM_MAX_WAIT);
    // }
    // UART_RFD900X << "GPS Valid: " << valid << stream::crlf;
    //
    // //
    // imu_icm20948.begin(to_digital(luna::pins::spi::cs::icm20948), Spi4);
    //
    // while (imu_icm20948.status != ICM_20948_Stat_Ok) {
    //     UART_RFD900X << imu_icm20948.statusString() << stream::crlf;
    //     delay(500);
    // }

    // Task initialization
    // dispatcher << (task_type([]() -> void { UART_RFD900X << "Called from lambda" << stream::crlf; }, nullptr, 500ul, millis), true)
    //            << task_type(fsm_map, nullptr, 1000ul, millis);

    // Low power mode
    // https://github.com/stm32duino/STM32LowPower/blob/main/README.md
    LowPower.begin();
    LowPower.enableWakeupFrom(&UART_RFD900X, []() -> void { UART_RFD900X << "I woke up!" << stream::crlf; });

    pvalid >> UART_RFD900X << stream::crlf;

    UART_RFD900X << "Init successful!" << stream::crlf;
}

void loop() {
    dispatcher();
    // static smart_delay sd(500, millis);
    //
    // if (sd) {
    //     gpio_write << io_function::toggle(luna::pins::gpio::LED_R)
    //                << io_function::toggle(luna::pins::gpio::LED_G)
    //                << io_function::toggle(luna::pins::gpio::LED_B)
    //                << io_function::toggle(luna::pins::gpio::BUZZER);
    //     UART_RFD900X << "Time: " << millis() << stream::crlf;
    //
    //     const auto vref = internal::read_vref();
    //     const auto temp = internal::read_cpu_temp(vref);
    //     ms1.read();
    //     ms2.read();
    //
    //     String s;
    //     csv_stream_crlf(s) << "";
    //
    //     { csv_stream_crlf(UART_RFD900X) << temp << ms1.getTemperature() << ms1.getPressure() << ms2.getTemperature() << ms2.getPressure(); }
    // }
}

void read_core() {
}

void read_gnss() {
}

void read_ms() {
}

void read_icm20948() {
}

void read_icm42688() {
}

void accept_command() {
}

void transmit_data() {
}

void save_data() {
}

void fsm_map() {
    UART_RFD900X << "Called from function" << stream::crlf;
    gpio_write << io_function::toggle(luna::pins::pyro::SIG_C);
}
