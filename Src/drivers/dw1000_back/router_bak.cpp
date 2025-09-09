/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include <stdio.h>
#include <stdint.h>
#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include "stm32f1xx_hal.h"

/* Initializing the anchorIDS */
const uint8_t anchor_ids[] = ANCHOR_IDS;
double anchor_distances[] = ANCHOR_IDS;

/* Frames used in the ranging process. See NOTE 2 below. */
const uint8_t RX_ANCHOR_ID_IND = 8;
const uint8_t TX_ANCHOR_ID_IND = 6;

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define FRAME_LEN_MAX 127

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2750
/* This is the delay from the end of the frame transmission to
the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300

/* Hold copies of computed time of flight and distance here for
reference so that it can be examined at a debug breakpoint. */

static uint32_t last_transition_ms = 0;
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};

int DW1000::reset() {
    if (common_reset() != 0) return -1;

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    return 0;
}

void DW1000::spin() {
    /* Check if enough time has passed since last ranging cycle */
    if (HAL_GetTick() - last_transition_ms < tx_delay_ms) {
        return;
    }

    /* Handle state machine transitions */
    switch (current_state) {
        case RangingState::IDLE:
            /* Start new ranging cycle */
            start_router_ranging_cycle();
            break;

        case RangingState::SENDING_RESPONSE:
            /* Send response message */
            send_response_message();
            break;

        case RangingState::PROCESSING_RESULT:
            /* Process ranging result */
            process_ranging_result();
            break;

        default:
            /* Check for timeouts in other states */
            if (is_timeout_elapsed(100)) {  // 100 ms timeout
                // logger.log("Timeout\n");
                start_next_ranging_cycle();
            }
            break;
    }
}

void DW1000::start_router_ranging_cycle() {
    /* Clear reception timeout to start next ranging process */
    dwt_setrxtimeout(0);

    /* Reset event flags */
    rx_complete = false;
    rx_timeout = false;
    rx_error = false;
    tx_complete = false;

    /* Activate reception immediately */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Change state to waiting for poll */
    change_state(RangingState::WAITING_POLL);
}

void DW1000::send_response_message() {
    uint32_t resp_tx_time;
    int ret;

    /* Retrieve poll reception timestamp */
    ranging_data.poll_rx_ts = get_rx_timestamp_u64();

    /* Set send time for response */
    resp_tx_time = (ranging_data.poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    /* After TX, expect final message with defined delay and timeout */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    /* Update message with current anchor ID */
    tx_resp_msg[TX_ANCHOR_ID_IND] = current_anchor_id;

    /* Write and send the response message */
    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret == DWT_ERROR) {
        // logger.log("F\n");
        start_next_ranging_cycle();
    } else {
        // logger.log("B\n");
        /* Change state to sending response */
        change_state(RangingState::SENDING_RESPONSE);
        frame_seq_nb++;
    }
    last_transition_ms = HAL_GetTick();
}

/* Helper functions are defined in dw1000.cpp to avoid multiple definitions */
