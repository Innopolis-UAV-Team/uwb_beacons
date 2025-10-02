/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "logger.hpp"
#include <string.h>
#include <logs.h>

Logger logger;

void logs(const char *s) {
    logger.log(s);
}

void logc(const uint8_t *s, uint16_t len) {
    logger.log(s, len);
}

void Logger::init() {
    usart_init();
}

void Logger::log(const char* s) {
    usart_send((uint8_t*)s, strlen(s));
}

void Logger::log(const uint8_t* s, uint16_t len) {
    usart_send((uint8_t*)s, len);
}

void Logger::spin() {
    usart_run();
}
