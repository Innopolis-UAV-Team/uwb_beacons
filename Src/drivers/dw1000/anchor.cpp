/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"

/* Delay between frames, in UWB microseconds.*/
/* This is the delay from the end of the frame transmission to
the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 3750


static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

#define SOURCE_ID_IND    8
#define DEST_ID_IND      6

/* Frames used in the ranging process. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', ID, 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', ID, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
// TODO(asiiapine): change to len 22
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', ID,
                                0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int DW1000::reset() {
    if (common_reset() != 0) return -1;
    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    return 0;
}

int8_t DW1000::spin() {
    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    static uint32_t last_transition_ms = 0;
    if (HAL_GetTick() - last_transition_ms < RNG_DELAY_MS) {
        update_status(ModuleState::MODULE_IDLE);
        return 0;
    }
    last_transition_ms = HAL_GetTick();
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
        * set by dwt_setrxaftertxdelay() has elapsed. */
    int res = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    if (res != 0) {
        update_status(ModuleState::MODULE_ERROR);
        return res;
    }

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    uint8_t n_tries = 0;
    bool got_response = false;
    while ((!got_response) & (n_tries < 3)) {
        n_tries++;

        if (read_message() != 0) {
            /* Activate reception immediately. */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        uint8_t id = rx_buffer[SOURCE_ID_IND];
        rx_resp_msg[SOURCE_ID_IND] = id;
        tx_final_msg[DEST_ID_IND] = id;
        if (rx_buffer[DEST_ID_IND] != ID) {
            continue;
        }
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
            continue;
        }
        got_response = true;
    }
    if (!got_response) {
        update_status(ModuleState::MODULE_ERROR);
        return -1;
    }

/* Check that the frame is the expected response from the companion "DS TWR responder" example. */
    uint32 final_tx_time;
    int ret;

    /* Retrieve poll transmission and response reception timestamp. */
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();

    /* Compute final message transmission time. See NOTE 10 below. */
    final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
    final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    /* Write all timestamps in the final message. See NOTE 11 below. */
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

    /* Write and send final message. See NOTE 8 below. */
    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed
    to the next one.*/
    if (ret != DWT_SUCCESS) {
        update_status(ModuleState::MODULE_ERROR);
        return -1;
    }

    /* Increment frame sequence number after transmission of the final message (modulo 256). */
    frame_seq_nb++;
    /* Execute a delay between ranging exchanges. */
    update_status(ModuleState::MODULE_OPERATIONAL);
    return 0;
}
