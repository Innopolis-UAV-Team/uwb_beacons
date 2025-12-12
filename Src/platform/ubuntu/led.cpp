/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "peripheral/led/led.hpp"

#define BLUE_LED_Pin LED2_Pin
#define GREEN_LED_Pin LED3_Pin
#define RED_LED_Pin LED1_Pin

#define BLUE_LED_GPIO_Port LED2_GPIO_Port
#define GREEN_LED_GPIO_Port LED3_GPIO_Port
#define RED_LED_GPIO_Port LED1_GPIO_Port

bool leds_state[3] = {false, false, false};

LED led;

void LED::init() {
}

void LED::on(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            leds_state[i] = true;
        }
        return;
    }
    leds_state[color] = true;
}

void LED::off(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            leds_state[i] = false;
        }
        return;
    }
    leds_state[color] = false;
}

void LED::toggle(LED_COLOR color) {
    if (color == ALL) {
        for (int i = 0; i < 3; i++) {
            leds_state[i] = !leds_state[i];
        }
        return;
    }
    leds_state[color] = !leds_state[color];
}
