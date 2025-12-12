/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "logger.hpp"
#include <string.h>

Logger logger;
extern UsartManager usart_manager;

void logs(const char *s) {
    logger.log(s);
}

void logc(const uint8_t *s, uint16_t len) {
    logger.log(s, len);
}

void Logger::init() {
    usart_manager.init();
}

void Logger::log(const char* s) {
    usart_manager.send((uint8_t*)s, strlen(s));
}

void Logger::log(const uint8_t* s, uint16_t len) {
    usart_manager.send((uint8_t*)s, len);
}

void Logger::spin() {
    usart_manager.run();
}
