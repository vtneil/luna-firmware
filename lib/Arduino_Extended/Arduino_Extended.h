#ifndef ARDUINO_EXTENDED_H
#define ARDUINO_EXTENDED_H

#include <Arduino.h>
#include <stm32h7xx_ll_adc.h>

namespace traits {
    template<typename S>
    concept has_ostream = requires(S s, String str) {
        { s << str } -> std::same_as<S &>;
    };
}  // namespace traits

namespace stream {
    inline const char *crlf = "\r\n";

    namespace detail {
        class flush_type {};
    }  // namespace detail

    constexpr detail::flush_type flush = detail::flush_type();
}  // namespace stream

// Stream as iostream

template<typename T>
Stream &operator<<(Stream &stream, T &&s) {
    stream.print(std::forward<T>(s));
    return stream;
}

// String as ostream

template<typename T>
String &operator<<(String &string, T &&s) {
    string += std::forward<T>(s);
    return string;
}

namespace detail {
    template<traits::has_ostream OStream,
             size_t ReserveSize = 0,
             bool NewLine       = false>
    class csv_stream {
    private:
        OStream *m_stream = {};
        String m_string   = {};

    public:
        explicit csv_stream(OStream &stream)
            : m_stream(&stream) {
            m_string.reserve(ReserveSize);
        }

        csv_stream(const csv_stream &other)     = delete;
        csv_stream(csv_stream &&other) noexcept = delete;

        template<typename T>
        csv_stream &operator<<(T &&value) {
            m_string += std::forward<T>(value);
            m_string += ",";
            return *this;
        }

        ~csv_stream() {
            // End of message

            // Remove trailing comma
            m_string.remove(m_string.length() - 1);

            if constexpr (NewLine) {
                m_string += stream::crlf;
            }

            // Flush to stream
            *m_stream << m_string;
        }
    };
}  // namespace detail

/**
 * @tparam OStream Output Stream Type with "<<" stream operator
 * @tparam ReserveSize Internal string reserve size
 * @tparam NewLine Whether to insert CRLF at the end or not
 * @param stream Ouptut stream OStream object
 * @return Csv stream object
 */
template<traits::has_ostream OStream, size_t ReserveSize = 0, bool NewLine = false>
detail::csv_stream<OStream, ReserveSize, NewLine> csv_stream(OStream &stream) {
    return detail::csv_stream<OStream, ReserveSize, NewLine>(stream);
}

// IO Pin as stream

inline uint32_t to_digital(const PinName pin_name) {
    return pinNametoDigitalPin(pin_name);
}

namespace detail {
    struct IOFunction {};

    template<typename T>
    concept io_func = std::derived_from<T, IOFunction>;
}  // namespace detail

struct On : detail::IOFunction {
    PinName m_pin;

    explicit On(const PinName pin) : m_pin{pin} {}
};

struct Off : detail::IOFunction {
    PinName m_pin;

    explicit Off(const PinName pin) : m_pin{pin} {}
};

struct Toggle : detail::IOFunction {
    PinName m_pin;

    explicit Toggle(const PinName pin) : m_pin{pin} {}
};

namespace detail {
    // GPIO as ostream
    class io_direction_t {
    public:
        template<io_func IoFuncType>
        io_direction_t &operator<<(IoFuncType func) {
            if constexpr (std::is_same_v<IoFuncType, On>) {
                digitalWriteFast(func.m_pin, 1);
            } else if constexpr (std::is_same_v<IoFuncType, Off>) {
                digitalWriteFast(func.m_pin, 0);
            } else if constexpr (std::is_same_v<IoFuncType, Toggle>) {
                digitalToggle(func.m_pin);
            }
            return *this;
        }
    };

    // GPIO Config as stream
    class io_configuration_t {
    public:
        io_configuration_t &operator<<(const PinName pin) {
            pinMode(to_digital(pin), OUTPUT);
            return *this;
        }

        io_configuration_t &operator>>(const PinName pin) {
            pinMode(to_digital(pin), INPUT);
            return *this;
        }
    };
}  // namespace detail

inline detail::io_direction_t gpio_write;
inline detail::io_configuration_t gpio_config;

namespace internal {
    inline int32_t read_vref() {
        return __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_16B);
    }

    inline int32_t read_cpu_temp(const int32_t VRef) {
        return __LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION_16B);
    }
}  // namespace internal

#endif  //ARDUINO_EXTENDED_H
