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

// Device specific
#define THIS_DEVICE_ID (0)

// Type aliases

using time_type   = uint32_t;
using smart_delay = vt::smart_delay<time_type>;
using task_type   = vt::task_t<vt::smart_delay, time_type>;

template<size_t N>
using dispatcher_type = vt::task_dispatcher<N, vt::smart_delay, time_type>;

// Type defs

struct peripherals_t {
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
};

struct ms_ref_t {
    MS5611_SPI &ms;
    float &temp;
    float &pres;
    float &alt;

    ms_ref_t(MS5611_SPI &t_ms, float &t_temp, float &t_pres, float &t_alt) : ms{t_ms}, temp{t_temp}, pres{t_pres}, alt{t_alt} {}
};

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
HardwareSerial &USB_DEBUG    = Serial4;
HardwareSerial &UART_LORA    = Serial2;
HardwareSerial &UART_RFD900X = Serial4;

// Software data

luna::sensor_data_t sensor_data;
uint8_t rx_buffer[luna::config::MESSAGE_BUFFER_SIZE];

// Software filters
algorithm::KalmanFilter_1D kf[10];

// Communication data

luna::header_t tx_header = {
        {'<', '4', '5', '>'},
        THIS_DEVICE_ID,
        luna::payload_type::DATA,
        luna::transmit_direction::DOWNLINK,
        0,
        0, // Data size: must modify
        0  // Data checksum: must modify
};
luna::data_t tx_data;

luna::message_t tx_message(tx_header, &tx_data);

// Peripherals

peripherals_t pvalid;

SdExFat flash;
SdExFat sd;

FsUtil<SdExFat, ExFile> flash_util(flash);
FsUtil<SdExFat, ExFile> sd_util(sd);

SFE_UBLOX_GNSS gnss_m9v;
ICM_20948_SPI imu_icm20948;
ICM42688 imu_icm42688(SPI_1, to_digital(luna::pins::spi::cs::icm42688));
MS5611_SPI ms1(to_digital(luna::pins::spi::cs::ms1), &SPI_1);
MS5611_SPI ms2(to_digital(luna::pins::spi::cs::ms2), &SPI_4);

ms_ref_t ms1_ref = {ms1, sensor_data.ms1_temp, sensor_data.ms1_pres, sensor_data.ms1_alt};
ms_ref_t ms2_ref = {ms2, sensor_data.ms2_temp, sensor_data.ms2_pres, sensor_data.ms2_alt};

// Software control
dispatcher_type<20> dispatcher;
vt::fsm<luna::state_t> fsm(luna::state_t::STARTUP);
// HardwareTimer timer_led(TIM1);

extern void init_storage(SdExFat &sd_instance);

extern void read_continuity();

extern void read_core_temp();

extern void read_gnss();

extern void read_ms(ms_ref_t *ms);

extern void read_icm20948();

extern void read_icm42688();

extern void accept_command();

extern void transmit_data();

extern void debug();

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
             << luna::pins::gpio::BUZZER
             << luna::pins::power::PIN_24V;

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

    // analogWriteResolution(8);
    // tim_led.setOverflow(30, HERTZ_FORMAT);
    // tim_led.attachInterrupt([]() -> void {
    //     static union {
    //         struct {
    //             uint8_t pad;
    //             uint8_t led_r;
    //             uint8_t led_g;
    //             uint8_t led_b;
    //         };
    //         uint32_t val{};
    //     } pwm_val;
    //
    //     ++pwm_val.val;
    //     analogWrite(to_digital(luna::pins::gpio::LED_R), pwm_val.led_r);
    //     analogWrite(to_digital(luna::pins::gpio::LED_G), pwm_val.led_g);
    //     analogWrite(to_digital(luna::pins::gpio::LED_B), pwm_val.led_b);
    // });
    // tim_led.resume();

    // UART Interfaces
    // UART_USB.begin(UART_BAUD);
    UART_LORA.begin(luna::config::UART_BAUD);
    UART_RFD900X.begin(luna::config::RFD900X_BAUD);

    USB_DEBUG << "UART Hello!" << stream::crlf;

    // I2C (Wire) Interfaces
    Wire1.setClock(400000);
    Wire1.begin();

    // I2C Debug
    i2c_detect(USB_DEBUG, Wire1, 0, 127);

    // SPI Interfaces
    // SPI 1, 2, 3: 64 MHz; SPI 4,5: 120 MHz

    // Storage Initialization
    pvalid.flash = flash.begin(SdSpiConfig(luna::pins::spi::cs::flash,
                                           SHARED_SPI,
                                           SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                                           &SPI_3));
    if (pvalid.flash) { init_storage(flash); }

    pvalid.sd = sd.begin(SdSpiConfig(luna::pins::spi::cs::sd,
                                     SHARED_SPI,
                                     SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                                     &SPI_3));
    if (pvalid.sd) { init_storage(sd); }

    // Peripherals Initialization

    // Barometer and temperature
    pvalid.ms1 = ms1.begin();
    pvalid.ms2 = ms2.begin();

    // IMU
    pvalid.imu_icm42688 = imu_icm42688.begin() == 1;
    if (pvalid.imu_icm42688) {
        imu_icm42688.setAccelRange(ICM42688::ACCEL_RANGE_16G);
        imu_icm42688.setGyroRange(ICM42688::GYRO_RANGE_2000DPS);
        imu_icm42688.setFilters(true, true);
    }

    pvalid.imu_icm20948 = imu_icm20948.begin(to_digital(luna::pins::spi::cs::icm20948), SPI_4) == ICM_20948_Stat_Ok;
    if (pvalid.imu_icm20948) {
    } else {
        USB_DEBUG << imu_icm20948.statusString() << stream::crlf;
    }

    // GPS
    pvalid.gnss_m9v = gnss_m9v.begin(Wire1);
    if (pvalid.gnss_m9v) {
        gnss_m9v.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setAutoPVT(true, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);

        // Optional
        gnss_m9v.saveConfiguration(luna::config::UBLOX_CUSTOM_MAX_WAIT);
    }

    // Task initialization
    dispatcher << task_type(read_continuity, 1000ul, micros, 0)
               << task_type(fsm_map, 50ul * 1000ul, micros, 1)
               << (task_type(read_icm42688, 10ul, millis, 2), pvalid.imu_icm42688)
               << (task_type(read_ms, &ms1_ref, 100ul, millis, 2), pvalid.ms1)
               << (task_type(read_ms, &ms2_ref, 100ul, millis, 2), pvalid.ms2)
               << (task_type(read_gnss, 50ul, millis, 2), pvalid.gnss_m9v)
               << task_type(debug, 100ul, millis, 254)
               << task_type(read_core_temp, 500ul, millis, 255);

    // Low power mode
    // See details: https://github.com/stm32duino/STM32LowPower/blob/main/README.md
    LowPower.begin();
    LowPower.enableWakeupFrom(&USB_DEBUG, []() -> void {
        USB_DEBUG << "Wake up call triggered." << stream::crlf;
    });

    // Peripheral validation
    pvalid >> USB_DEBUG << stream::crlf;

    USB_DEBUG << "Init successful!" << stream::crlf;

    // Reset timers before starting
    dispatcher.reset();
}

void loop() { dispatcher(); }

void init_storage(SdExFat &sd_instance) {
}

void read_continuity() {
    static smart_delay sd(5000ul, millis);
    static smart_delay sd2(1000ul, millis);

    sensor_data.cont_a = gpio_read.sample<32>(luna::pins::pyro::SENS_A);
    sensor_data.cont_b = gpio_read.sample<32>(luna::pins::pyro::SENS_B);
    sensor_data.cont_c = gpio_read.sample<32>(luna::pins::pyro::SENS_C);

    static int prev    = gpio_read.sample<32>(luna::pins::gpio::USER_1);
    int meas           = gpio_read.sample<32>(luna::pins::gpio::USER_1);

    // todo: Very smart delay application (On phase, Off phase)

    if (meas && prev) {
        if (sd) {
            gpio_write << io_function::pull_high(luna::pins::pyro::SIG_A);
            gpio_write << io_function::pull_high(luna::pins::gpio::BUZZER);
            delay(1000);
        } else {
            if (sd2) {
                gpio_write << io_function::pull_high(luna::pins::gpio::BUZZER);
                delay(50);
            } else {
                gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);
            }
        }
    } else {
        gpio_write << io_function::pull_low(luna::pins::pyro::SIG_A);
        gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);
        sd.reset();
        sd2.reset();
    }

    prev = meas;
}

void read_core_temp() {
    sensor_data.cpu_temp = internal::read_cpu_temp(internal::read_vref());
}

void read_gnss() {
    if (gnss_m9v.getPVT(luna::config::UBLOX_CUSTOM_MAX_WAIT)) {
        sensor_data.timestamp = gnss_m9v.getUnixEpoch(luna::config::UBLOX_CUSTOM_MAX_WAIT);
        sensor_data.gps_siv   = gnss_m9v.getSIV(luna::config::UBLOX_CUSTOM_MAX_WAIT);
        sensor_data.gps_lat   = static_cast<double>(gnss_m9v.getLatitude(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        sensor_data.gps_lon   = static_cast<double>(gnss_m9v.getLongitude(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        sensor_data.gps_alt   = static_cast<float>(gnss_m9v.getAltitudeMSL(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
        // gnss_m9v.getMagAcc()
    }
}

void read_ms(ms_ref_t *ms) {
    ms->ms.read();
    ms->temp = ms->ms.getTemperature();
    ms->pres = ms->ms.getPressure();
    ms->alt  = pressure_altitude(ms->pres);
}

void read_icm20948() {
}

void read_icm42688() {
    imu_icm42688.readAccGyro(reinterpret_cast<double *>(&sensor_data.imu_1));
}

void accept_command(Stream *istream) {
    Stream &stream = *istream;  // Alias to istream
    uint32_t index = 0;

    if (stream.available()) {
        delay(20);
    }
    while (stream.available()) {
        rx_buffer[index++] = UART_RFD900X.read();
    }

    luna::header_t &rx_header = *reinterpret_cast<luna::header_t *>(rx_buffer);
    const luna::message_t rx_message(rx_header, rx_buffer + sizeof(luna::header_t));
    const luna::command_t rx_command = rx_message.get<luna::command_t>();
}

void transmit_data() {
}

void debug() {
    gnss_m9v.getTimeValid();

    csv_stream_crlf(USB_DEBUG)
            << gpio_read(luna::pins::gpio::BUZZER)
            << gpio_read(luna::pins::pyro::SIG_C)
            << millis()
            << "CPU_TEMP"
            << sensor_data.cpu_temp
            << "CONTINUITY"
            << sensor_data.cont_a
            << sensor_data.cont_b
            << sensor_data.cont_c
            << "IMU_1(ICM4X)"
            << sensor_data.imu_1.acc.x
            << sensor_data.imu_1.acc.y
            << sensor_data.imu_1.acc.z
            << sensor_data.imu_1.gyro.x
            << sensor_data.imu_1.gyro.y
            << sensor_data.imu_1.gyro.z
            << "MS1"
            << sensor_data.ms1_temp
            << sensor_data.ms1_pres
            << sensor_data.ms1_alt
            << "MS2"
            << sensor_data.ms2_temp
            << sensor_data.ms2_pres
            << sensor_data.ms2_alt
            << "GNSS(m9V)"
            << sensor_data.gps_siv
            << sensor_data.timestamp
            << String(sensor_data.gps_lat, 6)
            << String(sensor_data.gps_lon, 6)
            << String(sensor_data.gps_alt, 4)
            << -1;
}

void save_data() {
}

void fsm_map() {
}
