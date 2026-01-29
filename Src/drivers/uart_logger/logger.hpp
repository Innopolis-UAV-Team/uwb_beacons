/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_DRIVERS_UART_LOGGER_LOGGER_HPP_
#define SRC_DRIVERS_UART_LOGGER_LOGGER_HPP_

#include "peripheral/usart/usart_manager.hpp"

class Logger {
 public:
    void init();
    void log(const char* s);
    void log(const char* s, uint16_t len);
    void log(const uint8_t* s, uint16_t len);
    void spin();
};

extern Logger logger;

#endif  // SRC_DRIVERS_UART_LOGGER_LOGGER_HPP_
