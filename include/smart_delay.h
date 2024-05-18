#ifndef LUNA_FIRMWARE_SMART_DELAY_H
#define LUNA_FIRMWARE_SMART_DELAY_H

#include <cstdint>
#include <concepts>

namespace vt {
    template<std::integral TimeType>
    class smart_delay {
    public:
        using time_func_t = TimeType();

    private:
        time_func_t *m_func        = {};
        uint32_t m_target_interval = {};
        uint32_t m_prev_time       = {};
        uint32_t m_true_interval   = {};

    public:
        smart_delay(TimeType interval, time_func_t *time_func) : m_func{time_func}, m_target_interval{interval} {
            if (time_func != nullptr)
                m_prev_time = time_func();
        }

        smart_delay(const smart_delay &)     = default;
        smart_delay(smart_delay &&) noexcept = default;

        smart_delay &operator=(const smart_delay &other) {
            if (this == &other) {
                return *this;
            }

            m_func            = other.m_func;
            m_target_interval = other.m_target_interval;
            m_prev_time       = other.m_prev_time;
            m_true_interval   = other.m_true_interval;

            return *this;
        }

        smart_delay &operator=(smart_delay &&other) noexcept {
            m_func            = std::move(other.m_func);
            m_target_interval = std::move(other.m_target_interval);
            m_prev_time       = std::move(other.m_prev_time);
            m_true_interval   = std::move(other.m_true_interval);

            return *this;
        }

        bool operator()() {
            return operator bool();
        }

        explicit operator bool() {
            if (m_func == nullptr) {
                return false;
            }

            // Adaptive interval adjustment
            TimeType curr_time = m_func();
            if (curr_time - m_prev_time >= m_true_interval) {
                // absolute delta_e
                TimeType delta_e = m_target_interval > m_true_interval
                                           ? m_target_interval - m_true_interval
                                           : m_true_interval - m_target_interval;
                m_true_interval  = delta_e < m_true_interval
                                           ? m_true_interval - delta_e
                                           : m_true_interval + delta_e;
                m_prev_time      = curr_time;

                return true;
            }

            return false;
        }

        void reset() {
            if (m_func != nullptr) {
                m_prev_time = m_func();
            }
        }

        constexpr TimeType interval() const {
            return m_true_interval;
        }
    };
}  // namespace vt

#endif  //LUNA_FIRMWARE_SMART_DELAY_H
