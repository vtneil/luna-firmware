// #ifndef FREERTOS_WRAPPER_H
// #define FREERTOS_WRAPPER_H
//
// #include <STM32FreeRTOS.h>
//
// // FreeRTOS Wrappers
//
// namespace rtos {
//   namespace type {
//     typedef void (*Func)();
//     typedef void (*FuncParams)(void *);
//   }  // namespace type
//
//   inline void create_task(const type::Func task, const uint16_t stack_depth, const UBaseType_t priority) {
//     xTaskCreate(reinterpret_cast<TaskFunction_t>(task), "", stack_depth, nullptr, priority, nullptr);
//   }
//
//   inline void create_loop(const type::Func task, const uint16_t stack_depth, const UBaseType_t priority, const uint32_t delay_ms = 1) {
//     struct task_data {
//       const uint32_t delay;
//       type::Func     func_ptr;
//     };
//
//     if (!task)
//       return;
//
//     auto wrapper = [](void *_ptr_to_task) {
//       auto [delay, func] = *static_cast<task_data *>(_ptr_to_task);
//       for (;;) {
//         (*func)();
//         vTaskDelay(pdMS_TO_TICKS(delay));
//       }
//     };
//
//     // Allocate a copy
//     auto *data_cpy = new task_data{delay_ms, task};
//
//     if (xTaskCreate(wrapper, "", stack_depth, data_cpy, priority, nullptr) != pdPASS) {
//       // Cleanup if fails
//       delete data_cpy;
//     }
//   }
//
//   template<typename FuncParams>
//   void create_task(FuncParams task, const uint16_t stack_depth, void *const args, const UBaseType_t priority) {
//     xTaskCreate(reinterpret_cast<TaskFunction_t>(task), "", stack_depth, args, priority, nullptr);
//   }
//
//   template<typename FuncParams>
//   void create_loop(const FuncParams task, const uint16_t stack_depth, void *args, const UBaseType_t priority, const uint32_t delay_ms = 1) {
//     struct task_data {
//       const uint32_t   delay;
//       type::FuncParams func_ptr;
//       void            *args;
//     };
//
//     if (!task)
//       return;
//
//     auto wrapper = [](void *_ptr_to_task_data) {
//       auto [delay, func, args] = *static_cast<task_data *>(_ptr_to_task_data);
//       for (;;) {
//         (*func)(args);
//         vTaskDelay(pdMS_TO_TICKS(delay));
//       }
//     };
//
//     // Allocate a copy
//     auto *data_cpy = new task_data{delay_ms, reinterpret_cast<type::FuncParams>(task), args};
//
//     if (xTaskCreate(wrapper, "", stack_depth, data_cpy, priority, nullptr) != pdPASS) {
//       // Cleanup if fails
//       delete data_cpy;
//     }
//   }
//
//   inline void start() {
//     vTaskStartScheduler();
//   }
// }  // namespace rtos
//
// #endif  //FREERTOS_WRAPPER_H
