/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/


#include "application.hpp"
#include "drivers/dw1000/dw1000.hpp"
#include "drivers/uart_logger/logger.hpp"

extern IWDG_HandleTypeDef hiwdg;

__attribute__((noreturn)) void application_entry_point() {
    logger.init();
    dw1000.set_calibration(4, 6300);  // id, distance in mm
    if (dw1000.init() != 0) {
        logger.log("INIT FAILED\n");
        while (true) {
            logger.spin();
            HAL_GPIO_TogglePin(GPIOA, LED1_Pin);
            HAL_Delay(1000);
        }
    }

    HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
    while (true) {
        HAL_GPIO_TogglePin(GPIOA, LED2_Pin);
        logger.spin();
        auto res = dw1000.spin();
        if  (!dw1000.is_calibration) {
            if (res != 0) {
                HAL_GPIO_TogglePin(GPIOA, LED1_Pin);
                continue;
            }
        }
        HAL_IWDG_Refresh(&hiwdg);
    }
}
