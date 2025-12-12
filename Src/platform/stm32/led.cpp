/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "peripheral/led/led.hpp"
#include "main.h"
#include <stm32f103xb.h>


#define BLUE_LED_Pin LED2_Pin
#define GREEN_LED_Pin LED3_Pin
#define RED_LED_Pin LED1_Pin

#define BLUE_LED_GPIO_Port LED2_GPIO_Port
#define GREEN_LED_GPIO_Port LED3_GPIO_Port
#define RED_LED_GPIO_Port LED1_GPIO_Port

LED led;

struct LedPin {
    GPIO_TypeDef *port;
    uint16_t pin;
};

LedPin ledPins[] = {
        {BLUE_LED_GPIO_Port, BLUE_LED_Pin},
        {GREEN_LED_GPIO_Port, GREEN_LED_Pin},
        {RED_LED_GPIO_Port, RED_LED_Pin},
};


void LED::init() {
}

void LED::on(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            ledPins[i].port->BSRR = ledPins[i].pin;
        }
        return;
    }
    ledPins[color].port->BSRR = ledPins[color].pin;
}

void LED::off(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            ledPins[i].port->BRR = static_cast<uint32_t>(ledPins[i].pin) << 16u;
        }
        return;
    }
    ledPins[color].port->BRR = static_cast<uint32_t>(ledPins[color].pin) << 16u;
}

void LED::toggle_pin(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, static_cast<GPIO_PinState>(!(HAL_GPIO_ReadPin(port, pin))));
}

void LED::toggle(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            toggle_pin(ledPins[i].port, ledPins[i].pin);
        }
        return;
    }
    toggle_pin(ledPins[color].port, ledPins[color].pin);
}
