#include "application.hpp"
#include "main.h"

__weak __attribute__((noreturn)) void application_entry_point() {
    uint32_t next_blink_time_ms = 1000;

    while (true) {
        uint32_t crnt_time_ms = HAL_GetTick();
        if (crnt_time_ms < next_blink_time_ms) {
            continue;
        }
        next_blink_time_ms = crnt_time_ms + 1000;

        /*Configure GPIO pin Output Level */
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_TogglePin(GPIOA, LED2_Pin|LED1_Pin);
    }
}
