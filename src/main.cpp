/**
 * @file main.cpp
 * @author Vivatsathorn Thitasirivit
 * @date 13 May 2024
 * @brief LUNA Firmware
 */

#include "lib_xcore"
#include "dispatcher"
#include "vt_linalg"
#include "vt_kalman"
#include "avionics_algorithm.h"

#include "Arduino_Extended.h"
#include "File_Utility.h"
#include "freertos_wrapper.h"
#include <STM32LowPower.h>
#include <STM32RTC.h>

#include <Wire.h>
#include <SPI.h>

#include <SdFat.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <ICM_20948.h>
#include <ICM42688.h>
#include <MS5611_SPI.h>
#include <INA236.h>

#include "luna_pin_def.h"
#include "luna_state_def.h"
#include "luna_peripheral_def.h"
#include "luna_message.h"
#include "luna_protocol.h"

// Device specific
#define THIS_DEVICE_ID             (0)
#define THIS_FILE_PREFIX           "LUNA_LOGGER_0_"
#define THIS_FILE_EXTENSION        "CSV"
#define THIS_GEIGER_FILE_PREFIX    "LUNA_GEIGER_LOGGER_0_"
#define THIS_GEIGER_FILE_EXTENSION "BIN"

// #define HW_FORMAT_CARD

// Type aliases

using time_type    = uint32_t;
using smart_delay  = xcore::nonblocking_delay<time_type>;
using on_off_timer = xcore::on_off_timer<time_type>;
using task_type    = xcore::task_t<xcore::nonblocking_delay, time_type>;

template<size_t N>
using dispatcher_type         = xcore::task_dispatcher<N, xcore::nonblocking_delay, time_type>;

using sd_sd_t                 = SdExFat;
using sd_file_t               = ExFile;
using flash_sd_t              = SdFat32;
using flash_file_t            = File32;

constexpr size_t FILTER_ORDER = 4;
constexpr double dt_base      = 0.1;
constexpr double covariance   = 0.01;
constexpr double alpha        = 0.;
constexpr double beta         = 0.;
constexpr double G            = 9.81;

// Type defs

struct peripherals_t {
  union {
    struct {
      uint8_t ina236;
      uint8_t gnss_m9v;
      uint8_t imu_icm42688;
      uint8_t imu_icm20948;

      uint8_t ms1;
      uint8_t ms2;
      uint8_t flash;
      uint8_t sd;
    };

    uint8_t arr[8];
  };

  template<traits::has_ostream OStream>
  OStream &operator>>(OStream &ostream) {
    ostream << "INA236: " << ina236 << stream::crlf;
    ostream << "GNSS M9V: " << gnss_m9v << stream::crlf;
    ostream << "IMU ICM42688: " << imu_icm42688 << stream::crlf;
    ostream << "IMU ICM20948: " << imu_icm20948 << stream::crlf;
    ostream << "BARO MS1: " << ms1 << stream::crlf;
    ostream << "BARO MS2: " << ms2 << stream::crlf;
    ostream << "STOR FLASH: " << flash << stream::crlf;
    ostream << "STOR SD: " << sd << stream::crlf;
    return ostream;
  }
};

struct ms_ref_t {
  MS5611_SPI               &ms;
  float                    &temp;
  float                    &pres;
  float                    &alt;
  vt::kf_pos<FILTER_ORDER> &kf;

  ms_ref_t(MS5611_SPI &t_ms, float &t_temp, float &t_pres, float &t_alt, vt::kf_pos<FILTER_ORDER> &t_kf)
      : ms{t_ms}, temp{t_temp}, pres{t_pres}, alt{t_alt}, kf{t_kf} {}
};

// Serial

HardwareSerial Serial2(luna::pins::comm::lora::UART_RX, luna::pins::comm::lora::UART_TX);
HardwareSerial Serial4(luna::pins::comm::rfd900x::UART_RX, luna::pins::comm::rfd900x::UART_TX);
HardwareSerial Serial7(luna::pins::comm::raspi::UART_RX, luna::pins::comm::raspi::UART_TX);

// I2C

TwoWire Wire1(to_digital(luna::pins::i2c::ch1::SDA),
              to_digital(luna::pins::i2c::ch1::SCL));

// SPI

SPIClass SPI_1(to_digital(luna::pins::spi::ch1::MOSI),
               to_digital(luna::pins::spi::ch1::MISO),
               to_digital(luna::pins::spi::ch1::SCK));
SPIClass SPI_3(to_digital(luna::pins::spi::ch3::MOSI),
               to_digital(luna::pins::spi::ch3::MISO),
               to_digital(luna::pins::spi::ch3::SCK));
SPIClass SPI_4(to_digital(luna::pins::spi::ch4::MOSI),
               to_digital(luna::pins::spi::ch4::MISO),
               to_digital(luna::pins::spi::ch4::SCK));

// SD Card

SdSpiConfig flash_config(luna::pins::spi::cs::flash,
                         SHARED_SPI,
                         SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                         &SPI_3);
SdSpiConfig sd_config(luna::pins::spi::cs::sd,
                      SHARED_SPI,
                      SD_SCK_MHZ(luna::config::SD_SPI_CLOCK_MHZ),
                      &SPI_3);

// Buzzer Controller

on_off_timer::interval_params buzzer_intervals(luna::config::BUZZER_ON_INTERVAL,
                                               luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_IDLE_INTERVAL));
bool                          enable_buzzer = true;

// Hardware references

HardwareSerial &USB_DEBUG    = Serial4;
HardwareSerial &UART_RPI     = Serial7;
HardwareSerial &UART_RFD900X = Serial4;

// Software data

luna::sensor_data_t sensor_data;

// Software filters
struct software_filters {
  vt::kf_pos<FILTER_ORDER> ms1_pres{dt_base, covariance, alpha, beta};
  vt::kf_pos<FILTER_ORDER> ms2_pres{dt_base, covariance, alpha, beta};

  struct {
    luna::vec3_u<vt::kf_pos<4>> acc{dt_base, covariance, alpha, beta};
    luna::vec3_u<vt::kf_pos<4>> gyro{dt_base, covariance, alpha, beta};
  } imu_1, imu_2;

  vt::kf_pos<FILTER_ORDER> altitude{dt_base, covariance, alpha, beta};
  vt::kf_acc<FILTER_ORDER> acceleration{dt_base, covariance, alpha, beta};
} filters;

// Communication data

String    tx_data;
String    sd_data;

time_type tx_interval  = luna::config::TX_IDLE_INTERVAL;
time_type log_interval = luna::config::LOG_IDLE_INTERVAL;

// Peripherals

peripherals_t                    pvalid;

FsUtil<flash_sd_t, flash_file_t> flash_util;
FsUtil<sd_sd_t, sd_file_t>       sd_util;

INA236                           ina236(luna::config::INA236_ADDRESS, &Wire1);
SFE_UBLOX_GNSS                   gnss_m9v;
ICM_20948_SPI                    imu_icm20948;
ICM42688                         imu_icm42688(SPI_1, to_digital(luna::pins::spi::cs::icm42688));
MS5611_SPI                       ms1(to_digital(luna::pins::spi::cs::ms1), &SPI_1);
MS5611_SPI                       ms2(to_digital(luna::pins::spi::cs::ms2), &SPI_4);

ms_ref_t                         ms1_ref = {ms1, sensor_data.ms1_temp, sensor_data.ms1_pres, sensor_data.ms1_alt, filters.ms1_pres};
ms_ref_t                         ms2_ref = {ms2, sensor_data.ms2_temp, sensor_data.ms2_pres, sensor_data.ms2_alt, filters.ms2_pres};

luna::proto::raspi_cam           cam(UART_RPI);
uint8_t                          cam_state = 0;

// Software control
dispatcher_type<32> dispatcher;
bool                launch_override = false;

struct {
  uint8_t pressure     : 2 = 0b11;  // Use both sensors (averaging)
  uint8_t acceleration : 2 = 0b11;  // Use both sensors (averaging)
} sensor_mode;

inline const char *sensor_mode_string(const uint8_t mode) {
  switch (mode) {
    case 0b01:
      return "1";
    case 0b10:
      return "2";
    case 0b11:
      return "F";
    default:
      __builtin_unreachable();
  }
}

struct {
  float                altitude{};
  float                altitude_offset{};
  luna::vec3_u<double> acc;
} ground_truth;

HardwareTimer      timer_led(TIM3);
HardwareTimer      timer_buz(TIM4);
luna::pins::PwmLed pwm_led(timer_led, timer_buz);

// Geiger Counter Variables

constexpr int     GEIGER_READ_RESOLUTION           = 10;
constexpr size_t  NUM_GEIGER_BINS                  = 1 << GEIGER_READ_RESOLUTION;
volatile uint32_t geiger_val                       = 0;
uint32_t          geiger_spectrum[NUM_GEIGER_BINS] = {};
uint32_t          geiger_count                     = 0;
double            geiger_cpm                       = 0.;
String            geiger_filename;
sd_file_t         geiger_file;

template<typename SdType, typename FileType>
extern void init_storage(FsUtil<SdType, FileType> &sd_util_instance);

extern void read_continuity();

extern void read_internal();

extern void read_gnss();

extern void read_ms(ms_ref_t *ms);

extern void read_icm20948();

extern void read_icm42688();

extern void calculate_ground_truth();

extern void synchronize_kf();

extern void accept_command(HardwareSerial *istream);

extern void construct_data();

extern void transmit_data(time_type *interval_ms);

extern void save_data(time_type *interval_ms);

extern void fsm_eval();

extern void buzzer_led_control(on_off_timer::interval_params *intervals_ms);

extern void raspi_cam_uart_daemon();

extern void calculate_geiger();

extern void save_geiger_spectrum();

// Geiger counter add-on
void setup_geiger() {
  // Geiger counter setup
  pinMode(luna::pins::geiger::DIGITAL_READ, INPUT);
  pinMode(luna::pins::geiger::ANALOG_READ, INPUT_ANALOG);
  analogReadResolution(10);

  auto register_count = []() -> void {
    geiger_val = analogRead(luna::pins::geiger::ANALOG_READ);
  };

  const uint32_t d_pin     = to_digital(luna::pins::geiger::DIGITAL_READ);
  const uint32_t d_pin_int = digitalPinToInterrupt(d_pin);

  attachInterrupt(d_pin_int, register_count, RISING);

  // File setup
  geiger_filename = sd_util.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
  geiger_file     = sd_util.open<FsMode::WRITE>(geiger_filename);
}

void setup() {
  setup_geiger();

  // GPIO and Digital Pins
  dout_low << luna::pins::gpio::LED_R
           << luna::pins::gpio::LED_G
           << luna::pins::gpio::LED_B
           << luna::pins::pyro::SIG_A
           << luna::pins::pyro::SIG_B
           << luna::pins::pyro::SIG_C
           << luna::pins::power::PIN_24V;

  // LED setup
  pwm_led.set_range(32, 255);
  pwm_led.set_color(luna::MAGENTA);
  pwm_led.set_frequency(2);
  pwm_led.reset();

  // Buzzer setup
  timer_buz.setOverflow(luna::config::BUZZER_ON_INTERVAL * 1000, MICROSEC_FORMAT);
  timer_buz.attachInterrupt([]() -> void {
    gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);
    timer_buz.pause();
  });
  timer_buz.pause();

  // GPIO Configuration

  dout_low << luna::pins::gpio::BUZZER;

  din_config << luna::pins::pyro::SENS_A
             << luna::pins::pyro::SENS_B
             << luna::pins::pyro::SENS_C
             << luna::pins::gpio::USER_1
             << luna::pins::gpio::USER_2;

  // UART Interfaces

  UART_RPI.begin(luna::config::RPI_BAUD);
  UART_RFD900X.begin(luna::config::RFD900X_BAUD);

  // SPI Mode
  SPI_3.setDataMode(SPI_MODE0);

  SPI_1.begin();
  SPI_3.begin();
  SPI_4.begin();

  // I2C (Wire) Interfaces
  Wire1.setClock(400000);
  Wire1.begin();

  // I2C Debug
  i2c_detect(USB_DEBUG, Wire1, 0, 127);

  // SPI Interfaces
  // SPI 1, 2, 3: 64 MHz; SPI 4,5: 120 MHz

  // Storage Initialization
  pvalid.flash = flash_util.sd().begin(flash_config);
  if (pvalid.flash) { init_storage(flash_util); }


  pvalid.sd = sd_util.sd().begin(sd_config);
  if (pvalid.sd) { init_storage(sd_util); }


  // Peripherals Initialization

  // Battery voltage monitor
  pvalid.ina236 = ina236.begin();
  if (pvalid.ina236) {
    ina236.setADCRange(luna::config::INA236ADCRange::RANGE_80MV);
  }

  // Barometer and temperature
  pvalid.ms1 = ms1.begin();
  pvalid.ms2 = ms2.begin();

  float gnd  = 0.f;

  if (pvalid.ms1) {
    ms1.reset();
    ms1.setOversampling(OSR_STANDARD);
    for (size_t i = 0; i < 20; ++i) {
      read_ms(&ms1_ref);
    }
    gnd += sensor_data.ms1_alt;
  }

  if (pvalid.ms2) {
    ms2.reset();
    ms2.setOversampling(OSR_STANDARD);
    for (size_t i = 0; i < 20; ++i) {
      read_ms(&ms2_ref);
    }
    gnd += sensor_data.ms2_alt;
  }

  ground_truth.altitude_offset = gnd * 0.5f;

  // IMU
  pvalid.imu_icm42688 = imu_icm42688.begin() == 1;
  if (pvalid.imu_icm42688) {
  }

  pvalid.imu_icm20948 = imu_icm20948.begin(to_digital(luna::pins::spi::cs::icm20948), SPI_4) == ICM_20948_Stat_Ok;
  if (pvalid.imu_icm20948) {
    imu_icm20948.swReset();
    delay(250);
    imu_icm20948.sleep(false);
    imu_icm20948.lowPower(false);
    imu_icm20948.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous);
    imu_icm20948.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, {.a = gpm16, .g = dps2000});
    imu_icm20948.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, {.a = acc_d246bw_n265bw, .g = gyr_d196bw6_n229bw8});
    imu_icm20948.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
  }

  // GPS
  pvalid.gnss_m9v = gnss_m9v.begin(Wire1);
  if (pvalid.gnss_m9v) {
    // Basic configuration
    gnss_m9v.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
    gnss_m9v.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
    gnss_m9v.setAutoPVT(true, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
    gnss_m9v.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, luna::config::UBLOX_CUSTOM_MAX_WAIT);
  }

  auto pyro_cutoff = []() -> void {
    static smart_delay sd_a(luna::config::PYRO_ACTIVATE_INTERVAL, millis);
    static smart_delay sd_b(luna::config::PYRO_ACTIVATE_INTERVAL, millis);
    static smart_delay sd_c(luna::config::PYRO_ACTIVATE_INTERVAL, millis);
    static bool        flag_a = false;
    static bool        flag_b = false;
    static bool        flag_c = false;

    if (sensor_data.pyro_a == luna::pyro_state_t::FIRING) {
      if (!flag_a) {
        sd_a.reset();
        flag_a = true;
      } else {
        sd_a([]() -> void {
          gpio_write << io_function::pull_low(luna::pins::pyro::SIG_A);
          sensor_data.pyro_a = luna::pyro_state_t::FIRED;
          flag_a             = false;
        });
      }
    }

    if (sensor_data.pyro_b == luna::pyro_state_t::FIRING) {
      if (!flag_b) {
        sd_b.reset();
        flag_b = true;
      } else {
        sd_b([]() -> void {
          gpio_write << io_function::pull_low(luna::pins::pyro::SIG_B);
          sensor_data.pyro_b = luna::pyro_state_t::FIRED;
          flag_b             = false;
        });
      }
    }

    if (sensor_data.pyro_c == luna::pyro_state_t::FIRING) {
      if (!flag_c) {
        sd_c.reset();
        flag_c = true;
      } else {
        sd_c([]() -> void {
          gpio_write << io_function::pull_low(luna::pins::pyro::SIG_C);
          sensor_data.pyro_c = luna::pyro_state_t::FIRED;
          flag_c             = false;
        });
      }
    }
  };

  // Low power mode
  // See details: https://github.com/stm32duino/STM32LowPower/blob/main/README.md
  LowPower.begin();
  LowPower.enableWakeupFrom(&UART_RFD900X, []() -> void {
    accept_command(&UART_RFD900X);
  });

  // Peripheral validation
  pwm_led.disable();

  {
    constexpr auto blink_green = []() -> void {
      luna::pins::SET_LED(luna::BLUE);
      gpio_write << io_function::pull_high(luna::pins::gpio::BUZZER);
      delay(luna::config::BUZZER_ON_INTERVAL);
      luna::pins::SET_LED(luna::BLACK);
      gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);
      delay(250);
    };

    constexpr auto blink_red = []() -> void {
      luna::pins::SET_LED(luna::RED);
      gpio_write << io_function::pull_high(luna::pins::gpio::BUZZER);
      delay(luna::config::BUZZER_ON_INTERVAL * 5);
      luna::pins::SET_LED(luna::BLACK);
      gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);
      delay(250);
    };

    for (const uint8_t status: pvalid.arr) {
      if (status) {
        blink_green();
      } else {
        blink_red();
      }
    }
  }

  tx_data.reserve(512);
  sd_data.reserve(768);
  pvalid >> USB_DEBUG << stream::crlf;
  USB_DEBUG << "Init successful!" << stream::crlf;

  // Reset timers before starting
  pwm_led.set_frequency(luna::config::INTERVAL_MS_TO_HZ(luna::config::BUZZER_IDLE_INTERVAL));
  pwm_led.reset();

  gpio_write << io_function::pull_low(luna::pins::gpio::BUZZER);

  // Task initialization
  dispatcher << task_type(read_continuity, 500ul, micros, 0)
             << task_type(buzzer_led_control, &buzzer_intervals, 0)  // Adaptive
             << task_type(pyro_cutoff, 0)

             << task_type(calculate_ground_truth, 0)
             << task_type(fsm_eval, 25ul, millis, 0)

             << (task_type(read_icm42688, 25ul, millis, 1), pvalid.imu_icm42688)
             << (task_type(read_icm20948, 25ul, millis, 1), pvalid.imu_icm20948)
             << (task_type(read_ms, &ms1_ref, 25ul, millis, 1), pvalid.ms1)
             << (task_type(read_ms, &ms2_ref, 25ul, millis, 1), pvalid.ms2)
             << task_type(synchronize_kf, 25ul, millis, 1)

             << (task_type(read_gnss, 100ul, millis, 2), pvalid.gnss_m9v)

             << task_type(accept_command, &UART_RFD900X, 100ul, millis, 252)

             << task_type(construct_data, 25ul, millis, 253)
             << (task_type(save_data, &log_interval, 254), pvalid.sd)  // Adaptive
             << task_type(transmit_data, &tx_interval, 254)            // Adaptive
             << task_type(calculate_geiger, 254)

             << task_type(read_internal, 500ul, millis, 255);

  dispatcher.reset();
}

void loop() { dispatcher(); }

template<typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance) {
  sd_util_instance.find_file_name_one(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
  sd_util_instance.template open_one<FsMode::WRITE>();
}

void read_continuity() {
  sensor_data.cont_a = gpio_read.sample<128>(luna::pins::pyro::SENS_A);
  sensor_data.cont_b = gpio_read.sample<128>(luna::pins::pyro::SENS_B);
  sensor_data.cont_c = gpio_read.sample<128>(luna::pins::pyro::SENS_C);
}

void read_internal() {
  sensor_data.cpu_temp = internal::read_cpu_temp(internal::read_vref());
  sensor_data.batt_v   = ina236.getBusVoltage();
}

void read_gnss() {
  if (gnss_m9v.getPVT(luna::config::UBLOX_CUSTOM_MAX_WAIT)) {
    sensor_data.timestamp = gnss_m9v.getUnixEpoch(sensor_data.timestamp_us, luna::config::UBLOX_CUSTOM_MAX_WAIT);
    sensor_data.gps_siv   = gnss_m9v.getSIV(luna::config::UBLOX_CUSTOM_MAX_WAIT);
    sensor_data.gps_lat   = static_cast<double>(gnss_m9v.getLatitude(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    sensor_data.gps_lon   = static_cast<double>(gnss_m9v.getLongitude(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    sensor_data.gps_alt   = static_cast<float>(gnss_m9v.getAltitudeMSL(luna::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
  }
}

void read_ms(ms_ref_t *ms) {
  static uint32_t t_prev = millis();

  ms->ms.read();

  if (const float t = ms->ms.getTemperature(); t > 0.) {
    ms->temp = t;
  }

  if (const float p = ms->ms.getPressure(); p > 0.) {
    ms->kf.update_dt(millis() - t_prev);
    ms->kf.kf.predict().update(p);
    ms->pres = static_cast<float>(ms->kf.kf.state);
  }

  ms->alt = pressure_altitude(ms->pres);

  t_prev  = millis();
}

void read_icm20948() {
  static uint32_t t_prev = millis();

  if (imu_icm20948.dataReady()) {
    imu_icm20948.getAGMT();

    // !!! DIRECTION TUNED !!!
    // Acc: milli-G unit to mss
    sensor_data.imu_2.acc.x  = -imu_icm20948.accY() * (0.001 * G);
    sensor_data.imu_2.acc.y  = -imu_icm20948.accX() * (0.001 * G);
    sensor_data.imu_2.acc.z  = imu_icm20948.accZ() * (0.001 * G);
    sensor_data.imu_2.gyro.x = -imu_icm20948.gyrY();
    sensor_data.imu_2.gyro.y = -imu_icm20948.gyrX();
    sensor_data.imu_2.gyro.z = imu_icm20948.gyrZ();
  }

  for (size_t i = 0; i < 3; ++i) {
    const uint32_t dt = millis() - t_prev;
    filters.imu_2.acc.values[i].update_dt(dt);
    filters.imu_2.gyro.values[i].update_dt(dt);
    filters.imu_2.acc.values[i].kf.predict().update(sensor_data.imu_2.acc.values[i]);
    filters.imu_2.gyro.values[i].kf.predict().update(sensor_data.imu_2.gyro.values[i]);
    sensor_data.imu_2.acc.values[i]  = filters.imu_2.acc.values[i].kf.state;
    sensor_data.imu_2.gyro.values[i] = filters.imu_2.gyro.values[i].kf.state;
  }

  t_prev = millis();
}

void read_icm42688() {
  static uint32_t t_prev = millis();

  if (imu_icm42688.readSensor() > 0) {
    // !!! DIRECTION TUNED !!!
    sensor_data.imu_1.acc.x  = -imu_icm42688.getAccelX_mss();
    sensor_data.imu_1.acc.y  = -imu_icm42688.getAccelY_mss();
    sensor_data.imu_1.acc.z  = -imu_icm42688.getAccelZ_mss();
    sensor_data.imu_1.gyro.x = -imu_icm42688.getGyroX_dps();
    sensor_data.imu_1.gyro.y = -imu_icm42688.getGyroY_dps();
    sensor_data.imu_1.gyro.z = -imu_icm42688.getGyroZ_dps();
  }

  for (size_t i = 0; i < 3; ++i) {
    const uint32_t dt = millis() - t_prev;
    filters.imu_1.acc.values[i].update_dt(dt);
    filters.imu_1.gyro.values[i].update_dt(dt);
    filters.imu_1.acc.values[i].kf.predict().update(sensor_data.imu_1.acc.values[i]);
    filters.imu_1.gyro.values[i].kf.predict().update(sensor_data.imu_1.gyro.values[i]);
    sensor_data.imu_1.acc.values[i]  = filters.imu_1.acc.values[i].kf.state;
    sensor_data.imu_1.gyro.values[i] = filters.imu_1.gyro.values[i].kf.state;
  }

  t_prev = millis();
}

void calculate_ground_truth() {
  switch (sensor_mode.pressure) {
    case 0b01:
      ground_truth.altitude = sensor_data.ms1_alt;
      break;
    case 0b10:
      ground_truth.altitude = sensor_data.ms2_alt;
      break;
    case 0b11:
      ground_truth.altitude = (sensor_data.ms1_alt + sensor_data.ms2_alt) * 0.5f;
      break;
    default:
      __builtin_unreachable();
  }

  switch (sensor_mode.acceleration) {
    case 0b01:
      ground_truth.acc.x = sensor_data.imu_1.acc.x;
      ground_truth.acc.y = sensor_data.imu_1.acc.y;
      ground_truth.acc.z = sensor_data.imu_1.acc.z;
      break;
    case 0b10:
      ground_truth.acc.x = sensor_data.imu_2.acc.x;
      ground_truth.acc.y = sensor_data.imu_2.acc.y;
      ground_truth.acc.z = sensor_data.imu_2.acc.z;
      break;
    case 0b11:
      ground_truth.acc.x = (sensor_data.imu_1.acc.x + sensor_data.imu_2.acc.x) * 0.5;
      ground_truth.acc.y = (sensor_data.imu_1.acc.y + sensor_data.imu_2.acc.y) * 0.5;
      ground_truth.acc.z = (sensor_data.imu_1.acc.z + sensor_data.imu_2.acc.z) * 0.5;
      break;
    default:
      __builtin_unreachable();
  }
}

void synchronize_kf() {
  static uint32_t t_prev    = millis();

  const double    total_acc = algorithm::root_sum_square(ground_truth.acc.x, ground_truth.acc.y, ground_truth.acc.z);

  filters.altitude.update_dt(millis() - t_prev);
  filters.acceleration.update_dt(millis() - t_prev);
  filters.altitude.kf.predict().update(ground_truth.altitude);
  filters.acceleration.kf.predict().update(total_acc - G);

  t_prev = millis();
}

void accept_command(HardwareSerial *istream) {
  Stream &stream = *istream;  // Alias to istream

  if (!stream.available()) return;
  delay(20ul);

  String rx_message = "";
  rx_message.reserve(64);

  while (stream.available()) {
    rx_message += static_cast<char>(stream.read());
  }

  // Repeats for at least 5 times before continuing
  if (rx_message.substring(0, 4) != "cmd ") {
    // Return if cmd header is invalid
    ++sensor_data.last_nack;
    return;
  }

  String command = rx_message.substring(4);
  command.trim();

  ++sensor_data.last_ack;

  if (command == "ping" || command == "wake" || command == "on") {
    // <--- Maybe a wakeup command --->

  } else if (command == "arm") {
    // <--- Arming the rocket --->
    sensor_data.ps     = luna::state_t::ARMED;
    tx_interval        = luna::config::TX_ARMED_INTERVAL;
    log_interval       = luna::config::LOG_ARMED_INTERVAL;
    sensor_data.pyro_a = luna::pyro_state_t::ARMED;
    sensor_data.pyro_b = luna::pyro_state_t::ARMED;
    sensor_data.pyro_c = luna::pyro_state_t::ARMED;

  } else if (command == "disarm") {
    sensor_data.ps     = luna::state_t::IDLE_SAFE;
    tx_interval        = luna::config::TX_IDLE_INTERVAL;
    log_interval       = luna::config::LOG_IDLE_INTERVAL;
    sensor_data.pyro_a = luna::pyro_state_t::DISARMED;
    sensor_data.pyro_b = luna::pyro_state_t::DISARMED;
    sensor_data.pyro_c = luna::pyro_state_t::DISARMED;

  } else if (command == "pad") {
    // <--- Prelaunch operation --->
    // Must be armed first!
    if (sensor_data.ps == luna::state_t::ARMED) {
      sensor_data.ps = luna::state_t::PAD_PREOP;
      tx_interval    = luna::config::TX_PAD_PREOP_INTERVAL;
      log_interval   = luna::config::LOG_PAD_PREOP_INTERVAL;
    }

  } else if (command == "shutup") {
    enable_buzzer = !enable_buzzer;
    pwm_led.set_buzzer(enable_buzzer);
    pwm_led.reset();

  } else if (command == "manual-trigger-a") {
    if (sensor_data.ps != luna::state_t::IDLE_SAFE && sensor_data.ps != luna::state_t::RECOVERED_SAFE) {
      gpio_write << io_function::pull_high(luna::pins::pyro::SIG_A);
      sensor_data.pyro_a = luna::pyro_state_t::FIRING;
    }

  } else if (command == "manual-trigger-b") {
    if (sensor_data.ps != luna::state_t::IDLE_SAFE && sensor_data.ps != luna::state_t::RECOVERED_SAFE) {
      gpio_write << io_function::pull_high(luna::pins::pyro::SIG_B);
      sensor_data.pyro_b = luna::pyro_state_t::FIRING;
    }

  } else if (command == "manual-trigger-c") {
    if (sensor_data.ps != luna::state_t::IDLE_SAFE && sensor_data.ps != luna::state_t::RECOVERED_SAFE) {
      gpio_write << io_function::pull_high(luna::pins::pyro::SIG_C);
      sensor_data.pyro_c = luna::pyro_state_t::FIRING;
    }

  } else if (command == "mode-pres-1") {
    sensor_mode.pressure = 0b01;

  } else if (command == "mode-pres-2") {
    sensor_mode.pressure = 0b10;

  } else if (command == "mode-pres-f") {
    sensor_mode.pressure = 0b11;

  } else if (command == "mode-accel-1") {
    sensor_mode.acceleration = 0b01;

  } else if (command == "mode-accel-2") {
    sensor_mode.acceleration = 0b10;

  } else if (command == "mode-accel-f") {
    sensor_mode.acceleration = 0b11;

  } else if (command == "launch-override") {
    launch_override = true;

  } else if (command == "recover") {
    // <--- Rocket landing confirmed --->
    sensor_data.ps = luna::state_t::RECOVERED_SAFE;
    if (sensor_data.pyro_a != luna::pyro_state_t::FIRED)
      sensor_data.pyro_a = luna::pyro_state_t::DISARMED;
    if (sensor_data.pyro_b != luna::pyro_state_t::FIRED)
      sensor_data.pyro_b = luna::pyro_state_t::DISARMED;
    if (sensor_data.pyro_c != luna::pyro_state_t::FIRED)
      sensor_data.pyro_c = luna::pyro_state_t::DISARMED;

  } else if (command == "zero") {
    // <--- Zero barometric altitude --->
    read_ms(&ms1_ref);
    read_ms(&ms2_ref);
    switch (sensor_mode.pressure) {
      case 0b01:
        ground_truth.altitude_offset = sensor_data.ms1_alt;
        break;
      case 0b10:
        ground_truth.altitude_offset = sensor_data.ms2_alt;
        break;
      case 0b11:
        ground_truth.altitude_offset = (sensor_data.ms1_alt + sensor_data.ms2_alt) * 0.5f;
        break;
      default:
        __builtin_unreachable();
    }

  } else if (command == "sleep") {
    // <--- Put the device into deep sleep mode (power saving) --->
    pwm_led.disable();
    luna::pins::PINS_OFF();
    luna::pins::SET_LED(luna::BLUE);
    LowPower.deepSleep();
    pwm_led.reset();

  } else if (command == "shutdown") {
    // <--- Shutdown the device --->
    pwm_led.disable();
    luna::pins::PINS_OFF();
    luna::pins::SET_LED(luna::RED);

    if (pvalid.sd) {
      sd_util.close_one();
    }
    if (pvalid.flash) {
      flash_util.close_one();
    }

    LowPower.deepSleep();

    __NVIC_SystemReset();

  } else if (command == "reboot" || command == "restart") {
    // <--- Reboot/reset the device --->
    if (pvalid.sd) {
      sd_util.close_one();
    }
    if (pvalid.flash) {
      flash_util.close_one();
    }
    __NVIC_SystemReset();

  } else if (command == "clear") {
    // <--- Clear ack and nack flags --->
    sensor_data.last_ack  = 0;
    sensor_data.last_nack = 0;

  } else if (command == "cam-ping") {
    cam.ping();

  } else if (command == "cam-start") {
    cam.start_video();

  } else if (command == "cam-stop") {
    cam.stop_video();

  } else if (command == "cam-query") {
    cam.get_status();

  } else {
    // <--- Unknown command: send back nack --->
    ++sensor_data.last_nack;
    --sensor_data.last_ack;
  }
}

void construct_data() {
  tx_data = "";
  csv_stream_crlf(tx_data)
    << "<45>"
    << sensor_data.timestamp
    << sensor_data.timestamp_us
    << millis()
    << sensor_data.tx_pc++
    << state_string(sensor_data.ps)
    << sensor_mode_string(sensor_mode.pressure)
    << sensor_mode_string(sensor_mode.acceleration)

    << String(sensor_data.gps_lat, 6)
    << String(sensor_data.gps_lon, 6)
    << String(ground_truth.altitude, 4)

    << pyro_state_string(sensor_data.pyro_a)
    << pyro_state_string(sensor_data.pyro_b)
    << pyro_state_string(sensor_data.pyro_c)
    << sensor_data.cont_a
    << sensor_data.cont_b
    << sensor_data.cont_c

    << sensor_data.ms1_temp << sensor_data.ms1_pres << sensor_data.ms1_alt
    << sensor_data.ms2_temp << sensor_data.ms2_pres << sensor_data.ms2_alt

    << sensor_data.imu_1.acc.x << sensor_data.imu_1.acc.y << sensor_data.imu_1.acc.z
    << sensor_data.imu_1.gyro.x << sensor_data.imu_1.gyro.y << sensor_data.imu_1.gyro.z

    << sensor_data.imu_2.acc.x << sensor_data.imu_2.acc.y << sensor_data.imu_2.acc.z
    << sensor_data.imu_2.gyro.x << sensor_data.imu_2.gyro.y << sensor_data.imu_2.gyro.z

    << sensor_data.cpu_temp
    << sensor_data.batt_v
    << sensor_data.last_ack
    << sensor_data.last_nack;
}

void transmit_data(time_type *interval_ms) {
  static time_type   prev = *interval_ms;
  static smart_delay sd(*interval_ms, millis);

  if (prev != *interval_ms) {
    sd.set_interval(*interval_ms);
    prev = *interval_ms;
  }

  sd([&]() -> void {
    UART_RFD900X << tx_data;
  });
}

void save_data(time_type *interval_ms) {
  static time_type   prev = *interval_ms;
  static smart_delay sd(*interval_ms, millis);
  static smart_delay sd_save(1000, millis);

  if (prev != *interval_ms) {
    sd.set_interval(*interval_ms);
    prev = *interval_ms;
  }

  sd([&]() -> void {
    sd_util.file() << tx_data;
  });

  sd_save([&]() -> void {
    sd_util.flush_one();
  });
}

void fsm_eval() {
  static bool               state_satisfaction = false;
  static time_type          launched_time      = 0;
  static algorithm::Sampler sampler[2];

  const double              alt_x = filters.altitude.kf.state_vector[0] - ground_truth.altitude_offset;
  const double              vel_x = filters.altitude.kf.state_vector[1];
  const double              acc   = filters.acceleration.kf.state_vector[2];

  switch (sensor_data.ps) {
    case luna::state_t::STARTUP: {
      // Next: always transfer
      sensor_data.ps = luna::state_t::IDLE_SAFE;
      break;
    }
    case luna::state_t::IDLE_SAFE: {
      //  <--- Next: wait for uplink --->
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_IDLE_INTERVAL);
      break;
    }
    case luna::state_t::ARMED: {
      // <--- Next: wait for uplink --->
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_ARMED_INTERVAL);

      if (launch_override) {
        sensor_data.ps = luna::state_t::PAD_PREOP;
        tx_interval    = luna::config::TX_PAD_PREOP_INTERVAL;
        log_interval   = luna::config::LOG_PAD_PREOP_INTERVAL;
      }

      break;
    }
    case luna::state_t::PAD_PREOP: {
      // !!!!! Next: DETECT launch !!!!!
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_PAD_PREOP_INTERVAL);

      static on_off_timer tim(luna::config::alg::LAUNCH_TON / 2, luna::config::alg::LAUNCH_TON / 2, millis);

      if (!state_satisfaction) {
        sampler[0].add(acc >= luna::config::alg::LAUNCH_ACC);
        sampler[1].add(acc >= luna::config::alg::LAUNCH_ACC);

        tim.on_rising([&] {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset();
        });

        tim.on_falling([&] {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset();
        });
      }

      state_satisfaction |= launch_override;

      if (state_satisfaction) {
        launched_time      = millis();
        sensor_data.ps     = luna::state_t::POWERED;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval  = luna::config::TX_ASCEND_INTERVAL;
        log_interval = luna::config::LOG_ASCEND_INTERVAL;
      }

      break;
    }
    case luna::state_t::POWERED: {
      // !!!!! Next: DETECT motor burnout !!!!!
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_ASCEND_INTERVAL);

      static on_off_timer tim(luna::config::alg::BURNOUT_TON / 2, luna::config::alg::BURNOUT_TON / 2, millis);

      if (!state_satisfaction) {
        sampler[0].add(acc < luna::config::alg::LAUNCH_ACC);
        sampler[1].add(acc < luna::config::alg::LAUNCH_ACC);

        tim.on_rising([&] {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[0].reset();
        });

        tim.on_falling([&] {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[1].reset();
        });
      }

      state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_BURNOUT_MAX;

      if (state_satisfaction) {
        sensor_data.ps     = luna::state_t::COASTING;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
      }

      break;
    }
    case luna::state_t::COASTING: {
      // !!!!! Next: DETECT apogee !!!!!
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_ASCEND_INTERVAL);

      static on_off_timer tim(luna::config::alg::APOGEE_SLOW_TON / 2, luna::config::alg::APOGEE_SLOW_TON / 2, millis);

      if (!state_satisfaction) {
        sampler[0].add(vel_x <= luna::config::alg::APOGEE_VEL);
        sampler[1].add(vel_x <= luna::config::alg::APOGEE_VEL);

        tim.on_rising([&] {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_APOGEE_MIN;
          }
          sampler[0].reset();
        });

        tim.on_falling([&] {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_APOGEE_MIN;
          }
          sampler[1].reset();
        });

        state_satisfaction |= millis() - launched_time >= luna::config::TIME_TO_APOGEE_MAX;
      }

      if (state_satisfaction) {
        sensor_data.ps     = luna::state_t::DROGUE_DEPLOY;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
      }

      break;
    }
    case luna::state_t::DROGUE_DEPLOY: {
      // Next: activate and always transfer
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_DESCEND_INTERVAL);

      static bool fired      = false;

      if (!fired) {
        gpio_write << io_function::pull_high(luna::pins::pyro::SIG_A);
        sensor_data.pyro_a = luna::pyro_state_t::FIRING;
        fired              = true;
      } else if (sensor_data.pyro_a == luna::pyro_state_t::FIRED) {
        sensor_data.ps = luna::state_t::DROGUE_DESCEND;
      }

      break;
    }
    case luna::state_t::DROGUE_DESCEND: {
      // !!!!! Next: DETECT main deployment altitude !!!!!
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_DESCEND_INTERVAL);

      static on_off_timer tim(luna::config::alg::MAIN_DEPLOYMENT_TON / 2, luna::config::alg::MAIN_DEPLOYMENT_TON / 2, millis);

      if (!state_satisfaction) {
        sampler[0].add(alt_x <= luna::config::alg::MAIN_ALTITUDE);
        sampler[1].add(alt_x <= luna::config::alg::MAIN_ALTITUDE);

        tim.on_rising([&] {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset();
        });

        tim.on_falling([&] {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset();
        });
      }

      if (state_satisfaction) {
        sensor_data.ps     = luna::state_t::MAIN_DEPLOY;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval  = luna::config::TX_DESCEND_INTERVAL;
        log_interval = luna::config::LOG_DESCEND_INTERVAL;
      }

      break;
    }
    case luna::state_t::MAIN_DEPLOY: {
      // Next: activate and always transfer
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_DESCEND_INTERVAL);

      static bool fired      = false;

      if (!fired) {
        gpio_write << io_function::pull_high(luna::pins::pyro::SIG_B);
        sensor_data.pyro_b = luna::pyro_state_t::FIRING;
        fired              = true;
      } else if (sensor_data.pyro_b == luna::pyro_state_t::FIRED) {
        sensor_data.ps = luna::state_t::MAIN_DESCEND;
      }

      break;
    }
    case luna::state_t::MAIN_DESCEND: {
      // !!!!! Next: DETECT landing !!!!!
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_DESCEND_INTERVAL);

      static on_off_timer tim(luna::config::alg::LANDING_TON / 2, luna::config::alg::LANDING_TON / 2, millis);

      if (!state_satisfaction) {
        const bool stable = algorithm::is_zero(vel_x, 0.5);
        sampler[0].add(stable);
        sampler[1].add(stable);

        tim.on_rising([&] {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset();
        });

        tim.on_falling([&] {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset();
        });
      }

      if (state_satisfaction) {
        sensor_data.ps     = luna::state_t::LANDED;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval  = luna::config::TX_IDLE_INTERVAL;
        log_interval = luna::config::LOG_IDLE_INTERVAL;
      }

      break;
    }
    case luna::state_t::LANDED: {
      // <--- Next: wait for uplink --->
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_DESCEND_INTERVAL);

      break;
    }
    case luna::state_t::RECOVERED_SAFE: {
      // Sink state (requires reboot)
      buzzer_intervals.t_off = luna::config::BUZZER_OFF_INTERVAL(luna::config::BUZZER_IDLE_INTERVAL);

      do_nothing();
      break;
    }
    case luna::state_t::LOG_ONLY: {
      // <--- Next: wait for uplink --->
      break;
    }
    default:
      __builtin_unreachable();
  }
}

void buzzer_led_control(on_off_timer::interval_params *intervals_ms) {
  // Interval change keeper
  static time_type     prev_on    = intervals_ms->t_on;
  static time_type     prev_off   = intervals_ms->t_off;
  static luna::state_t prev_state = luna::state_t::STARTUP;

  static struct {
    luna::RGB_MASK value = luna::RGB_MASK::BLACK;
  } onboard_led;

  // On-off timer
  static on_off_timer timer(intervals_ms->t_on, intervals_ms->t_off, millis);

  if (prev_on != intervals_ms->t_on || prev_off != intervals_ms->t_off) {
    timer.set_interval_on(intervals_ms->t_on);
    timer.set_interval_off(intervals_ms->t_off);
    prev_on  = intervals_ms->t_on;
    prev_off = intervals_ms->t_off;

    pwm_led.set_frequency(luna::config::INTERVAL_MS_TO_HZ(intervals_ms->t_on + intervals_ms->t_off));
    pwm_led.reset();
  }

  if (prev_state != sensor_data.ps) {
    prev_state = sensor_data.ps;

    switch (sensor_data.ps) {
      case luna::state_t::IDLE_SAFE:
        onboard_led.value = luna::RGB_MASK::GREEN;
        break;

      case luna::state_t::ARMED:
        onboard_led.value = luna::RGB_MASK::RED;
        break;

      case luna::state_t::PAD_PREOP:
        onboard_led.value = luna::RGB_MASK::RED;
        break;

      case luna::state_t::LANDED:
        onboard_led.value = luna::RGB_MASK::BLUE;
        break;

      case luna::state_t::RECOVERED_SAFE:
        onboard_led.value = luna::RGB_MASK::GREEN;
        break;

      default:
        onboard_led.value = luna::RGB_MASK::WHITE;
        break;
    }

    if (sensor_data.ps == luna::state_t::ARMED) {
      pwm_led.set_color(onboard_led.value, luna::RGB_MASK::GREEN);
    } else {
      pwm_led.set_color(onboard_led.value);
    }

    if (sensor_data.ps == luna::state_t::PAD_PREOP) {
      pwm_led.set_buzzer(false);
    } else {
      pwm_led.set_buzzer(enable_buzzer);
    }

    pwm_led.reset();
  }
}

void raspi_cam_uart_daemon() {
  // On-off timer
  static on_off_timer timer(1000ul, 500ul, millis);

  // Rising:  Query Action
  timer.on_rising([&] {
    cam.get_status();
  });

  // Falling: Get from buffer
  timer.on_falling([&] {
    const uint8_t rx_v = cam.wait(0);

    if (rx_v != luna::proto::INVALID) {
      if (cam_state == luna::proto::STAT_RECORDING &&
          rx_v != luna::proto::STAT_RECORDING) {
        cam.start_video();
      } else if (cam_state == luna::proto::STAT_STANDBY &&
                 rx_v != luna::proto::STAT_STANDBY) {
        cam.stop_video();
      } else {
        cam.set_status(rx_v);
      }
    }
  });
}

void calculate_geiger() {
  static constexpr uint32_t SAMPLE_INTERVAL_MS = 5000;
  static constexpr uint32_t SAVE_INTERVAL_MS   = 5000;
  static smart_delay        when_sample(SAMPLE_INTERVAL_MS, millis);
  static smart_delay        when_save(SAVE_INTERVAL_MS, millis);

  if (geiger_val > 0) {
    ++geiger_spectrum[geiger_val];
    ++geiger_count;
  }

  when_sample([]() -> void {
    geiger_cpm = static_cast<double>(geiger_count) / static_cast<double>(SAMPLE_INTERVAL_MS);
    geiger_cpm *= 1000. * 60.;
    geiger_count = 0;
  });

  when_save(save_geiger_spectrum);
}

void save_geiger_spectrum() {
  for (size_t i = 0; i < NUM_GEIGER_BINS; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      const uint8_t b = reinterpret_cast<const uint8_t *>(geiger_spectrum + i)[j];
      geiger_file.write(b);
    }
    geiger_spectrum[i] = 0;
  }
  geiger_file.flush();
}
