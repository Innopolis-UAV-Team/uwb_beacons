/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include <stdio.h>
#include <Multilateration.h>
#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include <cstdio>

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
static double tof;
static double distance;

uint8_t log_data[9] = {0};
static uint64_t poll_rx_ts;  // tx|rx changed, partially transfered
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x23, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);

/*
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::rx_ok_cb(const dwt_cb_data_t *cb_data) {
    /* Perform manual RX re-enabling. See NOTE 5 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);

    /* TESTING BREAKPOINT LOCATION #1 */

    /* A frame has been received, copy it to our local buffer. See NOTE 6 below. */
    if (cb_data->datalength <= FRAME_LEN_MAX) {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    /* TESTING BREAKPOINT LOCATION #2 */
}

int DW1000::reset() {
    if (common_reset() != 0) return -1;

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    return 0;
}

void DW1000::spin() {
    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
            (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32_t frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        rx_buffer[ALL_MSG_SN_IDX] = 0;

        /* Check that the frame is a poll sent by which anchor.
         * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
        static uint32_t last_transition_ms = 0;
        if (HAL_GetTick() - last_transition_ms < RNG_DELAY_MS) {
            return;
        }
        uint8_t anchor_id = rx_buffer[RX_ANCHOR_ID_IND];

        rx_poll_msg[RX_ANCHOR_ID_IND] = anchor_id;
        tx_resp_msg[TX_ANCHOR_ID_IND] = anchor_id;
        rx_final_msg[RX_ANCHOR_ID_IND] = anchor_id;
        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
            uint32_t resp_tx_time;
            int ret;

            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Set send time for response. See NOTE 9 below. */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

            /* Write and send the response message. See NOTE 10 below.*/
            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange
            and proceed to the next one. See NOTE 11 below. */
            if (ret == DWT_ERROR) {
                logs("F\n");
                return;
            }

            /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                        (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

            /* Increment frame sequence number after transmission of the response message (modulo 256). */
            frame_seq_nb++;

            if (status_reg & SYS_STATUS_RXFCG) {
                /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                if (frame_len <= RX_BUF_LEN) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                /* Check that the frame is a final message sent by "anchor1".
                    * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
                    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    double Ra, Rb, Da, Db;
                    int64_t tof_dtu;

                    /* Retrieve response transmission and final reception timestamps. */
                    resp_tx_ts = get_tx_timestamp_u64();
                    final_rx_ts = get_rx_timestamp_u64();

                    /* Get timestamps embedded in the final message. */
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                    poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                    resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                    final_rx_ts_32 = (uint32_t)final_rx_ts;
                    Ra = (double)(resp_rx_ts - poll_tx_ts);
                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    Da = (double)(final_tx_ts - resp_rx_ts);
                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
                    tof = tof_dtu * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;
                    if (distance < 0) {
                        return;
                    }

                    /* Display computed distance on LCD. */
                    uint32_t dist = uint32_t(int(distance * 1000));
                    log_data[0] = anchor_id;
                    log_data[1] =  dist & 0xFF;
                    log_data[2] = (dist >> 8) & 0xFF;
                    log_data[3] = (dist >> 16) & 0xFF;
                    log_data[4] = (dist >> 24) & 0xFF;
                    log_data[5] = 0xFF;
                    log_data[6] = 0xFF;
                    log_data[7] = 0xFF;
                    log_data[8] = 0;
                    logc(log_data, 9);
                } else {
                /* Check that the frame is a poll sent by "anchor2".
                    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    /* Clear RX error/timeout events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                }
            }
        }
        last_transition_ms = HAL_GetTick();
    }
}

/*
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::rx_err_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data;
    /* Re-activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    /* TESTING BREAKPOINT LOCATION #3 */
}

/*
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)  {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}
