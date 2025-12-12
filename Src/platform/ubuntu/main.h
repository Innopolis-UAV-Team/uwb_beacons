/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#pragma once
#include <stdint.h>

typedef uint32_t GPIO_TypeDef;
enum HAL_RES {
    HAL_OK = 0,
    HAL_ERROR = 1,
};

enum GPIO_PinState: uint32_t {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1,
};

// extern "C" {
//     void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
// }
