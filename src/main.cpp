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
    algorithm::KalmanFilter_1D &kf;

    ms_ref_t(MS5611_SPI &t_ms, float &t_temp, float &t_pres, float &t_alt, algorithm::KalmanFilter_1D &t_kf)
        : ms{t_ms}, temp{t_temp}, pres{t_pres}, alt{t_alt}, kf{t_kf} {}
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
struct software_filters {
    struct {
        luna::axis_data_u<algorithm::KalmanFilter_1D> acc;
        luna::axis_data_u<algorithm::KalmanFilter_1D> gyro;
    } imu_1;

    algorithm::KalmanFilter_1D ms1_pres;
    algorithm::KalmanFilter_1D ms2_pres;
};

software_filters filters;

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

time_type tx_interval  = luna::config::TX_IDLE_INTERVAL;
time_type log_interval = luna::config::LOG_IDLE_INTERVAL;

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

ms_ref_t ms1_ref = {ms1, sensor_data.ms1_temp, sensor_data.ms1_pres, sensor_data.ms1_alt, filters.ms1_pres};
ms_ref_t ms2_ref = {ms2, sensor_data.ms2_temp, sensor_data.ms2_pres, sensor_data.ms2_alt, filters.ms2_pres};

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

extern void accept_command(Stream *istream);

extern void transmit_data(time_type *interval_ms);

extern void debug();

extern void save_data(time_type *interval_ms);

extern void fsm_eval();

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
        // Basic configuration
        gnss_m9v.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setAutoPVT(true, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);

        // Dead reckoning
        // ???

        // Optional
        gnss_m9v.saveConfiguration(luna::config::UBLOX_CUSTOM_MAX_WAIT);
    }

    // Task initialization
    dispatcher << task_type(read_continuity, 1000ul, micros, 0)

               << task_type(fsm_eval, 50ul * 1000ul, micros, 1)

               << (task_type(read_icm20948, 10ul * 1000ul, micros, 1), pvalid.imu_icm20948)
               << (task_type(read_icm42688, 10ul * 1000ul, micros, 1), pvalid.imu_icm42688)
               << (task_type(read_ms, &ms1_ref, 100ul, millis, 2), pvalid.ms1)
               << (task_type(read_ms, &ms2_ref, 100ul, millis, 2), pvalid.ms2)
               << (task_type(read_gnss, 50ul, millis, 2), pvalid.gnss_m9v)

               << task_type(transmit_data, &tx_interval, 0ul, millis, 253)  // Adaptive
               << task_type(save_data, &log_interval, 0ul, millis, 253)     // Adaptive

               << task_type(debug, 100ul, millis, 254)
               << task_type(read_core_temp, 500ul, millis, 255);

    // Low power mode
    // See details: https://github.com/stm32duino/STM32LowPower/blob/main/README.md
    LowPower.begin();
    LowPower.enableWakeupFrom(&UART_RFD900X, []() -> void {
        accept_command(&UART_RFD900X);
    });

    // Peripheral validation
    pvalid >> USB_DEBUG << stream::crlf;
    USB_DEBUG << "Init successful!" << stream::crlf;

    // States
    sensor_data.ps = luna::state_t::IDLE_SAFE;

    // Reset timers before starting
    dispatcher.reset();
}

void loop() { dispatcher(); }

void init_storage(SdExFat &sd_instance) {
    // todo: init file
}

void read_continuity() {
    sensor_data.cont_a = gpio_read.sample<128>(luna::pins::pyro::SENS_A);
    sensor_data.cont_b = gpio_read.sample<128>(luna::pins::pyro::SENS_B);
    sensor_data.cont_c = gpio_read.sample<128>(luna::pins::pyro::SENS_C);
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
    }
}

void read_ms(ms_ref_t *ms) {
    ms->ms.read();
    ms->temp = ms->ms.getTemperature();
    ms->kf << ms->ms.getPressure();
    ms->pres = ms->kf.x();
    ms->alt  = pressure_altitude(ms->pres);
}

void read_icm20948() {
}

void read_icm42688() {
    imu_icm42688.readAccGyro(reinterpret_cast<double *>(&sensor_data.imu_1));
    for (size_t i = 0; i < 3; ++i) {
        // Acc KF
        filters.imu_1.acc.values[i] << sensor_data.imu_1.acc.values[i];
        sensor_data.imu_1.acc.values[i] = filters.imu_1.acc.values[i].x();
        // Gyro KF
        filters.imu_1.gyro.values[i] << sensor_data.imu_1.gyro.values[i];
        sensor_data.imu_1.gyro.values[i] = filters.imu_1.gyro.values[i].x();
    }
}

void accept_command(Stream *istream) {
    Stream &stream = *istream;  // Alias to istream

    if (!stream.available()) return;
    delayMicroseconds(20ul * 1000ul);

    String rx_message = "";
    rx_message.reserve(64);

    while (stream.available()) {
        rx_message += static_cast<char>(stream.read());
    }

    // Repeats for at least 5 times before continuing
    if (rx_message.substring(0, 4) != "cmd ") return;

    String command = rx_message.substring(4);
    command.trim();

    ++sensor_data.last_ack;

    if (command == "ping" || command == "wake" || command == "on") {
        // <--- Maybe a wakeup command --->
    } else if (command == "arm") {
        // <--- Arming the rocket --->
        sensor_data.ps = luna::state_t::ARMED;
    } else if (command == "prelaunch") {
        // <--- Prelaunch operation --->
        sensor_data.ps = luna::state_t::PAD_PREOP;
    } else if (command == "recover") {
        // <--- Rocket landing confirmed --->
        sensor_data.ps = luna::state_t::RECOVERED_SAFE;
    } else if (command == "sleep") {
        // <--- Put the device into deep sleep mode (power saving) --->
        LowPower.deepSleep();
    } else if (command == "shutdown") {
        // <--- Shutdown the device --->
        LowPower.shutdown();
    } else if (command == "reboot" || command == "restart") {
        // <--- Reboot/reset the device --->
        if (pvalid.sd) {
            // close file
        }
        if (pvalid.flash) {
            // close file
        }
        __NVIC_SystemReset();
    } else if (command == "clear") {
        // <--- Clear ack and nack flags --->
        sensor_data.last_ack  = 0;
        sensor_data.last_nack = 0;
    } else {
        // <--- Unknown command: send back nack --->
        ++sensor_data.last_nack;
        --sensor_data.last_ack;
    }
}

void transmit_data(time_type *interval_ms) {
    static time_type prev = *interval_ms;
    static smart_delay sd(*interval_ms, millis);

    if (prev != *interval_ms) {
        sd.set_interval(*interval_ms);
        prev = *interval_ms;
    }

    if (!sd) return;

    csv_stream_crlf(UART_RFD900X)
            << "<45>"
            << sensor_data.timestamp
            << sensor_data.tx_pc++
            << luna::state_string(sensor_data.ps)
            << "PYRO" << sensor_data.cont_a << sensor_data.cont_b << sensor_data.cont_c
            << "CONT" << sensor_data.cont_a << sensor_data.cont_b << sensor_data.cont_c
            << "GPS" << sensor_data.gps_siv
            << String(sensor_data.gps_lat, 6)
            << String(sensor_data.gps_lon, 6)
            << String(sensor_data.gps_alt, 4)
            << "MS1" << sensor_data.ms1_temp << sensor_data.ms1_pres << sensor_data.ms1_alt
            << "MS2" << sensor_data.ms2_temp << sensor_data.ms2_pres << sensor_data.ms2_alt
            << "IMU1ACC" << sensor_data.imu_1.acc.x << sensor_data.imu_1.acc.y << sensor_data.imu_1.acc.z
            << "IMU1GYR" << sensor_data.imu_1.gyro.x << sensor_data.imu_1.gyro.y << sensor_data.imu_1.gyro.z
            << "CPU_T" << sensor_data.cpu_temp
            << sensor_data.last_ack;
}

void debug() {
    // DEBUG
}

void save_data(time_type *interval_ms) {
    static time_type prev = *interval_ms;
    static smart_delay sd(*interval_ms, millis);

    if (prev != *interval_ms) {
        sd.set_interval(*interval_ms);
        prev = *interval_ms;
    }

    if (!sd) return;

    // todo: Save to storage
}

void fsm_eval() {
    constexpr static time_type time_to_burnout = 10ul * 1000ul;  // 10 seconds overestimate
    constexpr static time_type time_to_apogee  = 30ul * 1000ul;  // 30 seconds overestimate

    static bool state_satisfaction             = false;

    static time_type launched_time             = 0;

    switch (sensor_data.ps) {
        case luna::state_t::STARTUP: {
            // Next: always transfer
            sensor_data.ps = luna::state_t::IDLE_SAFE;
            break;
        }
        case luna::state_t::IDLE_SAFE: {
            //  <--- Next: wait for uplink --->
            break;
        }
        case luna::state_t::ARMED: {
            // <--- Next: wait for uplink --->
            break;
        }
        case luna::state_t::PAD_PREOP: {
            // Next: DETECT launch
            if (!state_satisfaction) {
            }
            if (state_satisfaction) {
                launched_time      = millis();
                sensor_data.ps     = luna::state_t::POWERED;
                state_satisfaction = false;
                tx_interval        = luna::config::TX_ASCEND_INTERVAL;
                log_interval       = luna::config::LOG_ASCEND_INTERVAL;
            }
            break;
        }
        case luna::state_t::POWERED: {
            // Next: DETECT motor burnout
            if (!state_satisfaction) {
            }
            if (state_satisfaction) {
                sensor_data.ps     = luna::state_t::COASTING;
                state_satisfaction = false;
            }
            break;
        }
        case luna::state_t::COASTING: {
            // Next: DETECT apogee
            if (!state_satisfaction) {
            }
            if (state_satisfaction) {
                sensor_data.ps     = luna::state_t::DROGUE_DEPLOY;
                state_satisfaction = false;
            }
            break;
        }
        case luna::state_t::DROGUE_DEPLOY: {
            // Next: activate and always transfer
            sensor_data.ps = luna::state_t::DROGUE_DESCEND;
            break;
        }
        case luna::state_t::DROGUE_DESCEND: {
            // Next: DETECT main deployment altitude
            if (!state_satisfaction) {
            }
            if (state_satisfaction) {
                sensor_data.ps     = luna::state_t::MAIN_DEPLOY;
                state_satisfaction = false;
                tx_interval        = luna::config::TX_DESCEND_INTERVAL;
                log_interval       = luna::config::LOG_DESCEND_INTERVAL;
            }
            break;
        }
        case luna::state_t::MAIN_DEPLOY: {
            // Next: activate and always transfer
            sensor_data.ps = luna::state_t::MAIN_DESCEND;
            break;
        }
        case luna::state_t::MAIN_DESCEND: {
            // Next: DETECT landing
            if (!state_satisfaction) {
            }
            if (state_satisfaction) {
                sensor_data.ps     = luna::state_t::LANDED;
                state_satisfaction = false;
                tx_interval        = luna::config::TX_IDLE_INTERVAL;
                log_interval       = luna::config::LOG_IDLE_INTERVAL;
            }
            break;
        }
        case luna::state_t::LANDED: {
            // <--- Next: wait for uplink --->
            break;
        }
        case luna::state_t::RECOVERED_SAFE: {
            // Sink state (requires reboot)
            break;
        }
        default:
            __builtin_unreachable();
    }
}

void buzzer_control(smart_delay::on_off *intervals_ms) {
    // todo
    static time_type prev_on  = intervals_ms->t_on;
    static time_type prev_off = intervals_ms->t_off;
    static smart_delay sd_on(intervals_ms->t_on, millis);
    static smart_delay sd_off(intervals_ms->t_off, millis);
    static bool is_on = false;

    if (prev_on != intervals_ms->t_on || prev_off != intervals_ms->t_off) {
        sd_on.set_interval(intervals_ms->t_on);
        sd_off.set_interval(intervals_ms->t_off);
        prev_on  = intervals_ms->t_on;
        prev_off = intervals_ms->t_off;
    }

    if (is_on) {
        if (sd_on) {
            // do something
            is_on = false;
        }
    } else {
        if (sd_off) {
            // do something
            is_on = true;
        }
    }
}

void future_accept_command(Stream *istream) {
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

void future_manual_trigger() {
    static smart_delay sd(5000ul, millis);
    static smart_delay sd2(1000ul, millis);

    static int prev = gpio_read.sample<32>(luna::pins::gpio::USER_1);
    const int meas  = gpio_read.sample<32>(luna::pins::gpio::USER_1);

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
