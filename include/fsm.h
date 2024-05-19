#ifndef FSM_H
#define FSM_H

namespace vt {
    namespace detail {
        template<typename T>
        concept enum_type = std::is_enum_v<T>;
    }

    template<detail::enum_type StateType>
    class fsm {
    protected:
        StateType m_prev{};
        StateType m_state{};

    public:
        constexpr explicit fsm(const StateType initial_state)
            : m_prev{initial_state}, m_state{initial_state} {}

        [[nodiscard]] constexpr StateType state() const {
            return m_state;
        }

        [[nodiscard]] constexpr StateType prev() const {
            return m_prev;
        }

        void transfer_to(const StateType next_state) {
            m_prev  = m_state;
            m_state = next_state;
        }
    };
}  // namespace vt

#endif  //FSM_H
