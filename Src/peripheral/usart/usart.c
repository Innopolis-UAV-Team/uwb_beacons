/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include <peripheral/usart/usart.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
MessagesCircularBuffer buffer = {0};
uint8_t uart_tx_busy = 0;

void push_message(UART_Message message) {
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

void pop_last_message(UART_Message* message) {
    uint8_t id = 0;
    if (buffer.size == 0) {
        message->len = 0;
        return;
    }
    if (buffer.next_id < buffer.size) {
        id = UART_MAX_QUEUE_SIZE - buffer.size + buffer.next_id;
    } else {
        id = buffer.next_id - buffer.size + 1;
    }
    *message = buffer.messages[id];
    buffer.size--;
}

void usart_init() {
    buffer.size = 0;
    buffer.next_id = 0;
    for (int i = 0; i < UART_MAX_QUEUE_SIZE; i++) {
        memset(&buffer.messages[i], 0, sizeof(UART_Message));
    }
}

void usart_send(uint8_t *data, uint16_t len) {
    UART_Message message = {.len = len};
    memcpy(message.data, data, len);
    push_message(message);
}

void usart_run() {
    if ((buffer.size == 0) || uart_tx_busy) {
        return;
    }
    UART_Message message;
    pop_last_message(&message);
    uart_tx_busy = 1;
    HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart1, message.data, message.len);
    if (res != HAL_OK) {
        HAL_GPIO_TogglePin(GPIOA, LED1_Pin);
        HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
        return;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    uart_tx_busy = 0;
    HAL_GPIO_TogglePin(GPIOA, LED2_Pin);
}
