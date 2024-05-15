#ifndef LUNA_FIRMWARE_TASKS_H
#define LUNA_FIRMWARE_TASKS_H

#include <cstdlib>
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

    template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType = uint32_t>
        requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    class tasks;

    template<template<typename> class SmartDelay, std::integral TimeType = uint32_t>
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
        task_t()                             = delete;
        constexpr task_t(const task_t &)     = default;
        constexpr task_t(task_t &&) noexcept = default;

        template<detail::task_fn TaskFuncType>
        constexpr task_t(TaskFuncType *task_func, void *arg, TimeType interval, time_func_t time_func, priority_t priority = 0)
            : m_func{reinterpret_cast<func_ptr<void *> *>(task_func)}, m_arg(arg),
              m_sd{smart_delay_t(interval, time_func)}, m_priority{priority} {}

        void operator()() {
            if ((m_func != nullptr) && m_sd) {
                m_func(m_arg);
            }
        }

        void reset() {
            m_sd.reset();
        }
    };

    template<size_t MaxTasks, template<typename> class SmartDelay, std::integral TimeType>
        requires traits::smart_delay_variant<SmartDelay<TimeType>, TimeType>
    class tasks {
        static_assert(MaxTasks > 0, "Scheduler size cannot be zero.");

    private:
        using task_t             = task_t<SmartDelay, TimeType>;

        size_t m_size            = {};
        task_t m_tasks[MaxTasks] = {};

    public:
        tasks()                  = default;
        tasks(const tasks &)     = delete;
        tasks(tasks &&) noexcept = delete;

        tasks &operator<<(task_t &&task) {
            if (m_size < MaxTasks) {
                size_t i;
                for (i = 0; i < m_size && m_tasks[i].m_priority < task.m_priority; ++i)
                    ;
                for (size_t j = 0; j < m_size - i; ++j) {
                    m_tasks[m_size - j] = m_tasks[m_size - j - 1];
                }
                m_tasks[i] = task;
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
