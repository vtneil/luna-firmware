#ifndef LUNA_FIRMWARE_TASKS_H
#define LUNA_FIRMWARE_TASKS_H

#include <cstdint>
#include <concepts>
#include <type_traits>

namespace vt {
    namespace traits {
        template<typename C, typename T>
        concept smart_delay_variant = requires(T value, T (*func_ptr)()) {
            { C(value, func_ptr) } -> std::same_as<C>;
            { static_cast<bool>(std::declval<C>()) } -> std::same_as<bool>;
            { std::declval<C>().reset() } -> std::same_as<void>;
        } && std::integral<T>;
    }  // namespace traits

    namespace detail {
        template<typename T>
        concept void_fn = std::is_same_v<T, void()>;

        template<typename T>
        concept void_fn_arg = std::is_same_v<T, void(void *)>;

        template<typename T>
        concept task_fn = detail::void_fn<T> || detail::void_fn_arg<T>;
    }  // namespace detail

    template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType>
        requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    class task_dispatcher;

    template<template<typename> class SmartDelay, std::integral TimeType>
        requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    class task_t {
    public:
        template<typename Arg>
        using func_ptr    = void(Arg);
        using time_func_t = TimeType();
        using priority_t  = uint8_t;

        template<size_t MaxTasks, template<typename> class, std::integral>
            requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
        friend class tasks;

    private:
        using smart_delay_t = SmartDelay<TimeType>;

        func_ptr<void *> *m_func;
        void *m_arg;
        smart_delay_t m_sd;
        priority_t m_priority;

    public:
        task_t()
            : m_func{nullptr}, m_arg(nullptr),
              m_sd{smart_delay_t(0, nullptr)}, m_priority{0} {}

        task_t(const task_t &)     = default;
        task_t(task_t &&) noexcept = default;

        template<detail::task_fn TaskFuncType>
        task_t(TaskFuncType *task_func, void *arg, TimeType interval, time_func_t time_func, priority_t priority = 0)
            : m_func{reinterpret_cast<func_ptr<void *> *>(task_func)}, m_arg(arg),
              m_sd{smart_delay_t(interval, time_func)}, m_priority{priority} {}

        task_t &operator=(const task_t &other) {
            if (this == &other) {
                return *this;
            }

            m_func     = other.m_func;
            m_arg      = other.m_arg;
            m_sd       = other.m_sd;
            m_priority = other.m_priority;

            return *this;
        }

        task_t &operator=(task_t &&other) noexcept {
            m_func     = std::move(other.m_func);
            m_arg      = std::move(other.m_arg);
            m_sd       = std::move(other.m_sd);
            m_priority = std::move(other.m_priority);

            return *this;
        }

        void operator()() {
            if (m_func != nullptr && m_sd) {
                m_func(m_arg);
            }
        }

        void reset() {
            m_sd.reset();
        }
    };

    template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType>
        requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    class task_dispatcher {
        static_assert(MaxTasks > 0, "Scheduler size cannot be zero.");

    private:
        using Task             = task_t<SmartDelay, TimeType>;

        size_t m_size          = {};
        Task m_tasks[MaxTasks] = {};

    public:
        task_dispatcher()                            = default;
        task_dispatcher(const task_dispatcher &)     = default;
        task_dispatcher(task_dispatcher &&) noexcept = default;

        task_dispatcher &operator+=(Task &&task) {
            return this->operator<<(std::forward<Task>(task));
        }

        task_dispatcher &operator<<(Task &&task) {
            if (m_size < MaxTasks) {
                size_t i;
                for (i = 0; i < m_size && m_tasks[i].m_priority < task.m_priority; ++i)
                    ;
                for (size_t j = 0; j < m_size - i; ++j) {
                    m_tasks[m_size - j] = std::move(m_tasks[m_size - j - 1]);
                }
                m_tasks[i] = std::move(task);
                ++m_size;
            }

            return *this;
        }

        void operator()() {
            for (size_t i = 0; i < m_size; ++i) {
                m_tasks[i]();
            }
        }

        void clear() {
            m_size = 0;
        }
    };
}  // namespace vt

#endif  //LUNA_FIRMWARE_TASKS_H
