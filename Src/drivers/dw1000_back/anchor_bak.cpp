/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include "stm32f1xx_hal.h"

/* Delay between frames, in UWB microseconds.*/
/* This is the delay from the end of the frame transmission to
the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Frames used in the ranging process. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x01, 0x21, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x01,
                                0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int DW1000::reset() {
    if (common_reset() != 0) return -1;

    return 0;
}

void DW1000::spin() {
    static uint32_t last_transition_ms = 0;

    // /* Check if enough time has passed since last ranging cycle */
    // if (HAL_GetTick() - last_transition_ms < RNG_DELAY_MS) {
    //     return;
    // }
    if (HAL_GetTick() - last_transition_ms < tx_delay_ms) {
        return;
    }

    /* Handle state machine transitions */
    switch (current_state) {
        case RangingState::IDLE:
            /* Start new ranging cycle */
            start_anchor_ranging_cycle();
            last_transition_ms = HAL_GetTick();
            tx_delay_ms = 0;
            break;

        case RangingState::SENDING_FINAL:
            /* Send final message */
            send_final_message();
            break;

        case RangingState::PROCESSING_RESULT:
            /* Process ranging result */
            process_ranging_result();
            break;

        default:
            /* Check for timeouts in other states */
            if (is_timeout_elapsed(5000)) { // 5 second timeout
                start_next_ranging_cycle();
            }
            break;
    }
}

void DW1000::start_anchor_ranging_cycle() {
    /* Reset event flags */
    tx_complete = false;
    rx_complete = false;
    rx_timeout = false;
    rx_error = false;

    /* Write frame data to DW1000 and prepare transmission */
    tx_poll_msg[ALL_MSG_SN_IDX] = 0x1;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    /* Start transmission */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* Change state to sending poll */
    change_state(RangingState::SENDING_POLL);

    /* Increment frame sequence number */
    frame_seq_nb++;
}

void DW1000::send_final_message() {
    uint32_t final_tx_time;
    int ret;

    /* Retrieve poll transmission and response reception timestamp */
    ranging_data.poll_tx_ts = get_tx_timestamp_u64();
    ranging_data.resp_rx_ts = get_rx_timestamp_u64();

    /* Compute final message transmission time */
    final_tx_time = (ranging_data.resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay */
    ranging_data.final_tx_ts = ((static_cast<uint64_t>(final_tx_time & 0xFFFFFFFEUL)) << 8)
                                + TX_ANT_DLY;

    /* Write all timestamps in the final message */
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], ranging_data.poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], ranging_data.resp_rx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], ranging_data.final_tx_ts);

    /* Write and send final message */
    // TODO: add anchor ID here, not sequence number
    tx_final_msg[ALL_MSG_SN_IDX] = 0x1;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);
    ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret == DWT_SUCCESS) {
        /* Change state to sending final */
        change_state(RangingState::SENDING_FINAL);
        frame_seq_nb++;
    } else {
        /* Error sending final message, restart cycle */
        start_next_ranging_cycle();
    }
}
