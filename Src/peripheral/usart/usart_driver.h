/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Copyright 2025 Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int usart_send(uint8_t *data, uint16_t len);
void usart_tx_isr();

#ifdef __cplusplus
}
#endif
