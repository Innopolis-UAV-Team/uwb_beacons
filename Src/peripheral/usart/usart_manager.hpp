/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/


#ifndef SRC_PERIPHERAL_USART_USART_MANAGER_HPP_
#define SRC_PERIPHERAL_USART_USART_MANAGER_HPP_

#include <stdint.h>
#include <main.h>

#define UART_MAX_QUEUE_SIZE 15
#define UART_MAX_MESSAGE_LEN 40

typedef struct {
    uint8_t data[UART_MAX_MESSAGE_LEN];
    uint8_t len;
} UART_Message;

typedef struct {
    uint8_t size;
    uint8_t next_id;  // tail, next free position
    UART_Message messages[UART_MAX_QUEUE_SIZE];
} MessagesCircularBuffer;


class UsartManager {
 public:
    void init();
    void send(uint8_t *data, uint8_t len);
    void run();
    static uint8_t tx_busy;
 private:
    uint8_t last_id;
    void pop_last_message(UART_Message* message);
    void push_message(UART_Message message);
};

extern UsartManager usart_manager;

#endif  // SRC_PERIPHERAL_USART_USART_MANAGER_HPP_
