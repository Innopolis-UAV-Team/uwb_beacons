/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "peripheral/usart/usart_manager.hpp"
#include "peripheral/usart/usart_driver.hpp"
#include <string.h>
#include "peripheral/led/led.hpp"

UsartManager usart_manager;
MessagesCircularBuffer buffer;
MessagesCircularBuffer rx_buffer;
uint8_t UsartManager::tx_busy = 0;

void MessagesCircularBuffer::push_message(UART_Message message) {
    memcpy(&messages[next_id], &message, sizeof(UART_Message));
    next_id++;
    size++;
    if (next_id >= UART_MAX_QUEUE_SIZE) {
        next_id = 0;
    }
    if (size >= UART_MAX_QUEUE_SIZE) {
        size = UART_MAX_QUEUE_SIZE;
    }
}

void MessagesCircularBuffer::pop_last_message(UART_Message* message) {
    if (size == 0) {
        message->len = 0;
        return;
    }
    if (next_id < size) {
        head_id = UART_MAX_QUEUE_SIZE - size + next_id;
    } else {
        head_id = next_id - size;
    }
    *message = messages[head_id];
    size--;
}

void UsartManager::init() {
    tx_busy = 0;
    for (int i = 0; i < UART_MAX_QUEUE_SIZE; i++) {
        memset(&buffer.messages[i].buffer.data, 0, sizeof(UART_Message));
    }
}

void UsartManager::send(uint8_t *data, uint8_t len) {
    UART_Message message = {.buffer = {0}, .len = len};
    memcpy(message.buffer.data, data, len);
    buffer.push_message(message);
}

void UsartManager::run() {
    if ((buffer.size == 0) || tx_busy) {
        return;
    }
    UART_Message message;
    buffer.pop_last_message(&message);
    if (message.len == 0) return;
    tx_busy++;
    int res = HAL::usart_send(message.buffer.data, message.len);
    if (res != HAL_OK) {
        led.toggle(LED_COLOR::YELLOW);
        led.off(LED_COLOR::BLUE);
        tx_busy--;
        return;
    }
    memset(&buffer.messages[buffer.head_id].buffer.data, 0, sizeof(UART_Message));
    led.off(LED_COLOR::YELLOW);
}

void HAL::usart_tx_isr() {
    UsartManager::tx_busy--;
    led.toggle(LED_COLOR::BLUE);
}

void HAL::usart_rx_isr() {
    // rx_buffer.push_message(message);
    buffer.push_message(usart_rx_buffer);
}
