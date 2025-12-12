/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "peripheral/usart/usart_manager.hpp"
#include "peripheral/usart/usart_driver.h"
#include <string.h>
#include "peripheral/led/led.hpp"

UsartManager usart_manager;
MessagesCircularBuffer buffer;
uint8_t UsartManager::tx_busy = 0;

void UsartManager::push_message(UART_Message message) {
    memcpy(&buffer.messages[buffer.next_id], &message, sizeof(UART_Message));
    buffer.next_id++;
    buffer.size++;
    if (buffer.next_id >= UART_MAX_QUEUE_SIZE) {
        buffer.next_id = 0;
    }
    if (buffer.size >= UART_MAX_QUEUE_SIZE) {
        buffer.size = UART_MAX_QUEUE_SIZE;
    }
}

void UsartManager::pop_last_message(UART_Message* message) {
    if (buffer.size == 0) {
        message->len = 0;
        return;
    }
    if (buffer.next_id < buffer.size) {
        last_id = UART_MAX_QUEUE_SIZE - buffer.size + buffer.next_id;
    } else {
        last_id = buffer.next_id - buffer.size;
    }
    *message = buffer.messages[last_id];
    buffer.size--;
}

void UsartManager::init() {
    tx_busy = 0;
    last_id = 0;

    buffer.size = 0;
    buffer.next_id = 0;
    for (int i = 0; i < UART_MAX_QUEUE_SIZE; i++) {
        memset(&buffer.messages[i], 0, sizeof(UART_Message));
    }
}

void UsartManager::send(uint8_t *data, uint8_t len) {
    UART_Message message = {.data = {0}, .len = len};
    memcpy(message.data, data, len);
    push_message(message);
}

void UsartManager::run() {
    if ((buffer.size == 0) || tx_busy) {
        return;
    }
    UART_Message message;
    pop_last_message(&message);
    if (message.len == 0) return;
    tx_busy++;
    int res = usart_send(message.data, message.len);
    if (res != HAL_OK) {
        led.toggle(LED_COLOR::RED);
        led.off(LED_COLOR::BLUE);
        tx_busy--;
        return;
    }
    memset(&buffer.messages[last_id], 0, sizeof(UART_Message));
    led.off(LED_COLOR::RED);
}

void usart_tx_isr() {
    UsartManager::tx_busy--;
    led.toggle(LED_COLOR::BLUE);
}
