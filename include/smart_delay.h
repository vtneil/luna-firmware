#ifndef LUNA_FIRMWARE_SMART_DELAY_H
#define LUNA_FIRMWARE_SMART_DELAY_H

#include <cstdlib>
#include <cstdint>
#include <concepts>

namespace vt {
    template<std::integral TimeType = uint32_t>
    class smart_delay {
    public:
        using time_func_t = TimeType();

    private:
        time_func_t *m_func        = {};
        uint32_t m_target_interval = {};
        uint32_t m_prev_time       = {};
        uint32_t m_true_interval   = {};

    public:
        constexpr smart_delay(TimeType interval, time_func_t *time_func) : m_func{time_func}, m_target_interval{interval} {
            if (time_func != nullptr)
                m_prev_time = time_func();
        }

        constexpr smart_delay(const smart_delay &)     = default;
        constexpr smart_delay(smart_delay &&) noexcept = default;

        explicit operator bool() {
            TimeType curr_time = 0;
            if (m_func != nullptr) {
                curr_time = m_func();
            }
            // Adaptive interval adjustment
            if (curr_time - m_prev_time >= m_true_interval) {
                // absolute delta_e
                TimeType delta_e = (m_target_interval > m_true_interval)
                                           ? m_target_interval - m_true_interval
                                           : m_true_interval - m_target_interval;
                if (delta_e < m_true_interval) {
                    m_true_interval -= delta_e;
                } else {
                    m_true_interval += delta_e;
                }
                m_prev_time = curr_time;
                return true;
            }
            return false;
        }

        void reset() {
            if (m_func != nullptr) {
                m_prev_time = m_func();
            }
        }
    };
}  // namespace vt

#endif  //LUNA_FIRMWARE_SMART_DELAY_H
