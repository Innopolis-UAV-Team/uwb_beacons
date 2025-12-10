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
#include "common.hpp"

#ifdef ANCHOR
#define APP_NAME "Anchor V1.0"
#else
#define APP_NAME "Rover V2.0"
#endif

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24

enum ModuleState: uint8_t {
    MODULE_IDLE = 0,
    MODULE_OPERATIONAL,
    MODULE_ERROR,
};

class DW1000 {
 public:
    ModuleState state;
    int init() {
        setup_DW1000RSTnIRQ(1);
        /* Display application name on LCD. */
        logger.log(APP_NAME);
        /* Reset and initialise DW1000.
         * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
         * performance. */
        if (reset() != 0) {
            state = ModuleState::MODULE_ERROR;
            return -1;
        }
        initialized = true;
        state = ModuleState::MODULE_OPERATIONAL;
        return 0;
    }
    int8_t spin();
    void set_calibration(int id, uint16_t real_distance_mm);
    bool is_calibration;
    int dwt_estimate_tx_time(uint16_t framelength, bool only_rmarker = false);

 private:
    int common_reset();
    int reset();
    int8_t read_message();
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    static uint32_t status_reg;
    static uint8_t rx_buffer[RX_BUF_LEN];

    /* Frame sequence number, incremented after each transmission. */
    static uint8_t frame_seq_nb;
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    static void rx_err_cb(const dwt_cb_data_t *cb_data);

    /* Data for calibration */
    uint16_t real_distance;
    uint16_t seeked_id = 0;
    double min_error = MAXFLOAT;
    uint32_t best_antenna_delay;
    antenna_delay_t antenna_delays = {TX_ANT_DLY, TX_ANT_DLY};
    uint16_t* antenna_delay = nullptr;
    void get_current_ant_delay();
    float seconds_to_dwt_s(float s) { return s * 512/499.2; }
    bool initialized = false;
};

extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
