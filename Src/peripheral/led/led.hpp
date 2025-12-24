/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#pragma once
#include <cstdint>
#include "main.h"

enum LED_COLOR {
    BLUE = 0,
    GREEN,
    YELLOW,
    ALL
};

class LED {
 public:
    void init();
    void on(LED_COLOR color);
    void off(LED_COLOR color);
    void toggle(LED_COLOR color);

 private:
    void set_color(LED_COLOR color);
    void toggle_pin(GPIO_TypeDef* port, uint16_t pin);
};

extern LED led;
