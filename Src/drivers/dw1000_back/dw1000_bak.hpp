/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_DRIVERS_DW1000_DW1000_HPP_
#define SRC_DRIVERS_DW1000_DW1000_HPP_

#include <string.h>
#include <stdint.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
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
    void setup_interrupts();

    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    static uint32_t status_reg;
    static uint8_t rx_buffer[RX_BUF_LEN];

    /* Frame sequence number, incremented after each transmission. */
    static uint8_t frame_seq_nb;
    
    /* Interrupt callback functions */
    static void rx_ok_cb(const dwt_cb_data_t *cb_data);
    static void rx_err_cb(const dwt_cb_data_t *cb_data);
    static void rx_to_cb(const dwt_cb_data_t *cb_data);
    static void tx_conf_cb(const dwt_cb_data_t *cb_data);
    
    /* State management for interrupt-based operation */
    static volatile bool tx_complete;
    static volatile bool rx_complete;
    static volatile bool rx_timeout;
    static volatile bool rx_error;
    static volatile uint32_t last_event_time;
    
    /* State machine for ranging process */
    enum class RangingState {
        IDLE,
        SENDING_POLL,           // Anchor: sending poll message
        WAITING_RESPONSE,       // Anchor: waiting for response
        SENDING_FINAL,          // Anchor: sending final message
        WAITING_POLL,           // Router: waiting for poll message
        SENDING_RESPONSE,       // Router: sending response
        WAITING_FINAL,          // Router: waiting for final message
        PROCESSING_RESULT       // Both: processing ranging result
    };
    
    static volatile RangingState current_state;
    static volatile uint32_t state_start_time;
    static volatile uint8_t current_anchor_id;
    
    /* Data storage for ranging results */
    struct RangingData {
        uint64_t poll_tx_ts;
        uint64_t resp_rx_ts;
        uint64_t final_tx_ts;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts;
        uint64_t final_rx_ts;
        uint8_t anchor_id;
        bool data_valid;
    };
    
    static RangingData ranging_data;
    
    /* Helper functions for state management and data processing */
    static void change_state(RangingState new_state);
    static void parse_received_frame();
    static void process_ranging_result();
    static void start_next_ranging_cycle();
    static bool is_timeout_elapsed(uint32_t timeout_ms);
    
    /* Anchor-specific functions */
    static void start_anchor_ranging_cycle();
    static void send_final_message();
    
    /* Router-specific functions */
    static void start_router_ranging_cycle();
    static void send_response_message();
};


extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
