/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "peripheral/usart/usart_driver.hpp"
#include "main.h"

extern UART_HandleTypeDef huart1;
UART_Message usart_rx_buffer = {};
UART_Message usart_tx_buffer = {};

int HAL::usart_send(uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart1, data, len);
    return res;
}

__weak void HAL::usart_tx_isr() {
    return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    HAL::usart_tx_isr();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(huart, usart_rx_buffer.buffer.data, UART_MAX_MESSAGE_LEN);
    HAL::usart_rx_isr();
}
