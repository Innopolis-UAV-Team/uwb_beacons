/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "peripheral/usart/usart_driver.h"
#include "application.hpp"


int usart_send(uint8_t *pData, uint16_t Size) {
    char buffer[Size];
    memcpy(buffer, pData, Size);
    printf("%s", buffer);
    usart_tx_isr();
    return 0;
}

__attribute__((weak)) void usart_tx_isr() {
    return;
}
