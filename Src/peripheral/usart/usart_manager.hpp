/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/


#ifndef SRC_PERIPHERAL_USART_USART_MANAGER_HPP_
#define SRC_PERIPHERAL_USART_USART_MANAGER_HPP_

#include <stdint.h>
#include <main.h>
#include "peripheral/usart/usart_driver.hpp"

#define UART_MAX_QUEUE_SIZE 15

class MessagesCircularBuffer {
    uint8_t next_id;  // tail, next free position
 public:
    uint8_t head_id;
    uint8_t size;
    UART_Message messages[UART_MAX_QUEUE_SIZE];
    void pop_last_message(UART_Message* message);
    void push_message(UART_Message message);
};


class UsartManager {
 public:
    void init();
    void send(uint8_t *data, uint8_t len);
    void run();
    static uint8_t tx_busy;
 private:
    uint8_t last_id;
};

extern UsartManager usart_manager;

#endif  // SRC_PERIPHERAL_USART_USART_MANAGER_HPP_
