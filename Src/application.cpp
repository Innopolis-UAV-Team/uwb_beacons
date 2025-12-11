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
    if (CALIBRATE) {
        dw1000.set_calibration(4, 6300);  // id, distance in mm
    }
    dw1000.init();

    HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
    while (true) {
        logger.spin();
        dw1000.spin();
        if  (!dw1000.is_calibration) {
            if (dw1000.state == ModuleState::MODULE_ERROR) {
                HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
                continue;
            }
            HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
        }
        HAL_IWDG_Refresh(&hiwdg);
    }
}
