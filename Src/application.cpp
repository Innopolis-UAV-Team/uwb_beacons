/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/


#include "application.hpp"
#include "drivers/dw1000/dw1000.hpp"
#include "drivers/uart_logger/logger.hpp"

__attribute__((noreturn)) void application_entry_point() {
    logger.init();
    if (dw1000.init() != 0) {
        logger.log("INIT FAILED");
        while (true) {
            logger.spin();
            HAL_GPIO_TogglePin(GPIOA, LED1_Pin);
            HAL_Delay(1000);
        }
    }
    while (true) {
        logger.spin();
        dw1000.spin();
    }
}
