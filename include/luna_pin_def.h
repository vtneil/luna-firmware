#ifndef LUNA_FIRMWARE_LUNA_PIN_DEF_H
#define LUNA_FIRMWARE_LUNA_PIN_DEF_H

#include <PinNames.h>
#include <Arduino_Extended.h>

namespace luna {
    enum RGB_MASK {
        BLACK = 0b000,
        BLUE,
        GREEN,
        CYAN,
        RED,
        MAGENTA,
        YELLOW,
        WHITE
    };
}

namespace luna::pins {
    namespace power {
        constexpr PinName PIN_24V = PB_12;
    }

    namespace gpio {
        constexpr PinName LED_R  = PE_13;  // Onboard RGB LED R
        constexpr PinName LED_G  = PE_12;  // Onboard RGB LED G
        constexpr PinName LED_B  = PE_14;  // Onboard RGB LED B
        constexpr PinName BUZZER = PD_9;   // Onboard buzzer
        constexpr PinName USER_1 = PF_9;   // Onboard push button 1
        constexpr PinName USER_2 = PF_8;   // Onboard push button 2
    }  // namespace gpio

    namespace pyro {
        constexpr PinName SIG_A  = PG_3;
        constexpr PinName SIG_B  = PG_4;
        constexpr PinName SIG_C  = PG_5;

        constexpr PinName SENS_A = PG_8;
        constexpr PinName SENS_B = PG_7;
        constexpr PinName SENS_C = PG_6;
    }  // namespace pyro

    namespace comm {
        namespace rpi {
            constexpr PinName CTS     = PB_14;
            constexpr PinName RTS     = PB_15;
            constexpr PinName UART_TX = PD_1;
            constexpr PinName UART_RX = PD_0;
        }  // namespace rpi

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
        // ch1: icm-42688, ms1
        namespace ch1 {
            constexpr PinName SCK  = PA_5;
            constexpr PinName MISO = PA_6;
            constexpr PinName MOSI = PA_7;
        }  // namespace ch1

        // ch3: sd, flash
        namespace ch3 {
            constexpr PinName SCK  = PC_10;
            constexpr PinName MISO = PC_11;
            constexpr PinName MOSI = PC_12;
        }  // namespace ch3

        // ch4: icm-20948, ms2
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

    namespace i2c::ch1 {
        constexpr PinName SDA = PB_7;
        constexpr PinName SCL = PB_8;
    }  // namespace i2c::ch1

    namespace tmc2209 {
        constexpr PinName UART_TX = PC_6;
        constexpr PinName UART_RX = PC_7;
    }  // namespace tmc2209

    constexpr auto SET_LED = [](const int color) {
        gpio_write << io_function::set(gpio::LED_R, BITS_AT(color, 2))
                   << io_function::set(gpio::LED_G, BITS_AT(color, 1))
                   << io_function::set(gpio::LED_B, BITS_AT(color, 0));
    };

    constexpr auto PINS_OFF = [] {
        gpio_write << io_function::pull_low(gpio::LED_R)
                   << io_function::pull_low(gpio::LED_G)
                   << io_function::pull_low(gpio::LED_B)
                   << io_function::pull_low(gpio::BUZZER)
                   << io_function::pull_low(pyro::SIG_A)
                   << io_function::pull_low(pyro::SIG_B)
                   << io_function::pull_low(pyro::SIG_C);
    };

    constexpr auto PINS_RESTORE = [] {
    };

    class PwmLed {
        HardwareTimer &timer_led;
        HardwareTimer &timer_buz;

        int num_colors       = 1;
        RGB_MASK m_colors[3] = {BLACK, BLACK, BLACK};
        uint32_t m_freq      = 1;
        uint8_t m_color_i    = 0;
        uint8_t pwm_val      = 0;
        uint8_t m_min        = 0;
        uint8_t m_max        = 255;
        bool direction       = true;
        bool m_buzzer        = false;

    public:
        explicit PwmLed(HardwareTimer &timer_led, HardwareTimer &timer_buz) : timer_led{timer_led}, timer_buz{timer_buz} {
            analogWriteResolution(8);
        }

        [[nodiscard]] constexpr bool is_max() const {
            return pwm_val == m_max;
        }

        [[nodiscard]] constexpr bool is_min() const {
            return pwm_val == m_min;
        }

        void set_range(const uint8_t min, const uint8_t max) {
            m_min = min;
            m_max = max;
        }

        void set_color(const RGB_MASK color) {
            m_color_i   = 0;
            num_colors  = 1;
            m_colors[0] = color;
        }

        void set_color(const RGB_MASK color1, const RGB_MASK color2) {
            m_color_i   = 0;
            num_colors  = 2;
            m_colors[0] = color1;
            m_colors[1] = color2;
        }

        void set_color(const RGB_MASK color1, const RGB_MASK color2, const RGB_MASK color3) {
            m_color_i   = 0;
            num_colors  = 3;
            m_colors[0] = color1;
            m_colors[1] = color2;
            m_colors[2] = color3;
        }

        void set_buzzer(const bool enable) {
            m_buzzer = enable;
        }

        void set_frequency(const uint32_t freq_hz) {
            m_freq = freq_hz;
        }

        void pause() {
            timer_led.pause();
        }

        void resume() {
            timer_led.resume();
        }

        void disable() {
            pause();
            dout_low << gpio::LED_R
                     << gpio::LED_G
                     << gpio::LED_B;
        }

        void reset() {
            disable();

            direction = true;
            pwm_val   = m_min;
            m_color_i = 0;

            timer_led.setOverflow((static_cast<uint32_t>(m_max - m_min) + 1) * m_freq * 2, HERTZ_FORMAT);

            timer_led.attachInterrupt([&] {
                if (direction)
                    ++pwm_val;
                else
                    --pwm_val;
                if (is_max()) {
                    direction = false;
                    if (m_buzzer) {
                        gpio_write << io_function::pull_high(gpio::BUZZER);
                        timer_buz.setCount(0);
                        timer_buz.resume();
                    }
                } else if (is_min()) {
                    direction = true;
                    if (m_color_i < num_colors - 1) {
                        ++m_color_i;
                    } else {
                        m_color_i = 0;
                    }
                    dout_low << gpio::LED_R
                             << gpio::LED_G
                             << gpio::LED_B;
                }

                const int color = m_colors[m_color_i];

                if (BITS_AT(color, 2))
                    gpio_write << io_function::pwm(gpio::LED_R, pwm_val);
                if (BITS_AT(color, 1))
                    gpio_write << io_function::pwm(gpio::LED_G, pwm_val);
                if (BITS_AT(color, 0))
                    gpio_write << io_function::pwm(gpio::LED_B, pwm_val);
            });

            resume();
        }
    };

}  // namespace luna::pins

#endif  //LUNA_FIRMWARE_LUNA_PIN_DEF_H
