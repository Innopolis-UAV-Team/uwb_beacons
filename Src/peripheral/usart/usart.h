/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_PERIPHERAL_USART_USART_H_
#define SRC_PERIPHERAL_USART_USART_H_

#include <stdint.h>
#include <main.h>

#ifdef __cplusplus
extern "C" {
#endif

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

void push_message(UART_Message message);

void pop_last_message(UART_Message* message);

void usart_init();
void usart_send(uint8_t *data, uint16_t len);
void usart_run();

#ifdef __cplusplus
}
#endif

#endif  // SRC_PERIPHERAL_USART_USART_H_
