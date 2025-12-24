/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#pragma once
#ifndef SRC_PERIPHERAL_USART_USART_DRIVER_H_
#define SRC_PERIPHERAL_USART_USART_DRIVER_H_

#include <stdint.h>

#define UART_MAX_MESSAGE_LEN 40
#define UART_MAX_QUEUE_SIZE 15

union Buffer {
    uint8_t data[UART_MAX_MESSAGE_LEN];
    char    str[UART_MAX_MESSAGE_LEN];
};

typedef struct {
    Buffer buffer;
    uint8_t len = 0;
} UART_Message;

extern UART_Message usart_rx_buffer;
extern UART_Message usart_tx_buffer;

namespace HAL {

int  usart_send(uint8_t *data, uint16_t len);
void usart_tx_isr();
void usart_rx_isr();
}


#endif  // SRC_PERIPHERAL_USART_USART_DRIVER_H_
