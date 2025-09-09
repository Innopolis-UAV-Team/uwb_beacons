/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_DRIVERS_DW1000_DW1000_HPP_
#define SRC_DRIVERS_DW1000_DW1000_HPP_

#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "stm32f103xb.h"
#include "../uart_logger/logger.hpp"
#include "port.h"

#define ANCHOR_IDS {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10}

#ifdef ANCHOR
#define APP_NAME "Anchor V1.0"
#else
#define APP_NAME "Rover V1.0"
#endif

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24

class DW1000 {
 public:
    int init() {
        setup_DW1000RSTnIRQ(1);
        /* Display application name on LCD. */
        logger.log(APP_NAME);
        /* Reset and initialise DW1000.
         * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
         * performance. */
        return reset();
    }
    void spin();

 private:
    int common_reset();
    int reset();

    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    static uint32_t status_reg;
    static uint8_t rx_buffer[RX_BUF_LEN];

    /* Frame sequence number, incremented after each transmission. */
    static uint8_t frame_seq_nb;
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    static void rx_err_cb(const dwt_cb_data_t *cb_data);
};


extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
