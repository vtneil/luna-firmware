#ifndef AVIONICS_ALGORITHM_H
#define AVIONICS_ALGORITHM_H

namespace algorithm {
    namespace detail {

    }

    namespace traits {
        template<typename F>
        concept GettableX = requires(F f) {
            { f.x() } -> std::same_as<double>;
        };
    }  // namespace traits

    class ExponentialMovingAverage {
    private:
        double m_alpha;
        double m_ema   = {};
        uint8_t m_init = {};

    public:
        explicit constexpr ExponentialMovingAverage(const double alpha)
            : m_alpha{alpha} {
        }

        ExponentialMovingAverage &operator<<(const double v) {
            if (!m_init) {
                m_ema  = v;
                m_init = 1;
            } else {
                m_ema = m_alpha * v + (1.0 - m_alpha) * m_ema;
            }
            return *this;
        }

        template<traits::GettableX KF_Type>
        ExponentialMovingAverage &operator<<(const KF_Type &kf_1d_object) {
            this->operator<<(kf_1d_object.x());
            return *this;
        }

        [[nodiscard]] constexpr double get() const {
            return m_ema;
        }

        void reset() {
            m_ema  = 0.0;
            m_init = 0;
        }
    };

    class KalmanFilter_1D {
    private:
        double m_x;  // Estimated state
        double m_P;  // Estimated error covariance
        double m_Q;  // Process noise covariance
        double m_R;  // Measurement noise covariance
        double m_K;  // Kalman gain

    public:
        KalmanFilter_1D(const double initial_x, const double initial_P, const double Q, const double R)
            : m_x(initial_x), m_P(initial_P), m_Q(Q), m_R(R), m_K(0.0) {
        }

        KalmanFilter_1D &predict(const double = 0.0) {
            m_P = m_P + m_Q;
            return *this;
        }

        KalmanFilter_1D &update(const double z) {
            m_K = m_P / (m_P + m_R);
            m_x = m_x + m_K * (z - m_x);
            m_P = (1 - m_K) * m_P;
            return *this;
        }

        KalmanFilter_1D &operator<<(const double z) {
            return predict().update(z);
        }

        KalmanFilter_1D &operator<<(const ExponentialMovingAverage &ema) {
            this->operator<<(ema.get());
            return *this;
        }

        [[nodiscard]] constexpr double x() const {
            return m_x;
        }

        [[nodiscard]] constexpr double P() const {
            return m_P;
        }

    public:
        static constexpr double initial_x     = 0.0;
        static constexpr double initial_P     = 1.0;
        static constexpr double initial_noise = 0.1;
    };

    class Kinematics {
    private:
        double m_dt;
        double m_x      = {};
        double m_v      = {};
        double m_a      = {};

        double m_prev_x = {};
        double m_prev_v = {};

        uint8_t m_cnt   = {};

    public:
        explicit constexpr Kinematics(const double dt) : m_dt{dt} {}

        Kinematics &operator<<(const double z) {
            switch (m_cnt) {
                case 0: {
                    m_x   = z;
                    m_cnt = 1;
                    break;
                }
                case 1: {
                    m_prev_x = m_x;
                    m_x      = z;
                    m_v      = (m_x - m_prev_x) / m_dt;
                    m_cnt    = 2;
                    break;
                }
                default: {
                    m_prev_v = m_v;
                    m_prev_x = m_x;
                    m_x      = z;
                    m_v      = (m_x - m_prev_x) / m_dt;
                    m_a      = (m_v - m_prev_v) / m_dt;
                    break;
                }
            }

            return *this;
        }

        [[nodiscard]] constexpr double x() const { return m_x; }
        [[nodiscard]] constexpr double v() const { return m_v; }
        [[nodiscard]] constexpr double z() const { return m_a; }
    };
}  // namespace algorithm

#endif  //AVIONICS_ALGORITHM_H
