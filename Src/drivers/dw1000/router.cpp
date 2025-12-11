/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include <stdio.h>
#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include <cstdio>
#include <cstring>
#include <logs.h>


/* Index to access some of the fields in the frames involved in the process. */
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define FRAME_LEN_MAX 127

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 5000
/* This is the delay from the end of the frame transmission to
the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3000 // , , 3500, 4000, 4500, 5000, 5500, 6000, 7000 does not work


/* Hold copies of computed time of flight and distance here for
reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

uint8_t log_data[9] = {0};
static uint64_t poll_rx_ts;  // tx|rx changed, partially transfered
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Frames used in the ranging process. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', ID, 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', ID, 'V', 0x00, 0x23, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern UART_HandleTypeDef huart1;
uint8_t data[50] = {0};

int DW1000::reset() {
    if (common_reset() != 0) return -1;

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    return 0;
}

int8_t DW1000::spin() {
    static uint8_t anchor_id;
    static double distance_sum = 0;
    static uint16_t n_attempts = 0;
    static uint16_t success_attempts = 0;
    if (is_calibration && (anchor_id == seeked_id)) {
        char buffer[UART_MAX_MESSAGE_LEN];
        n_attempts++;
        distance_sum += distance;
        if (abs(distance) >= 0.01f) {
            success_attempts++;
        }

        if (n_attempts > 100) {
            snprintf(buffer, sizeof(buffer), "CAL DLY: %d\n",
            static_cast<int>(*antenna_delay));
            *antenna_delay += 10;
            success_attempts = 0;
            n_attempts = 0;
            reset();
            if (success_attempts >= 10) {
                auto mean_value_mm = distance_sum * 1000/ success_attempts;
                auto error = abs(mean_value_mm - real_distance);

                if (min_error > error) {
                    min_error = error;
                    distance_sum = 0;
                    best_antenna_delay = *antenna_delay;
                    std::snprintf(buffer, sizeof(buffer), "DLY:%d ERR:%d\n",
                                                                static_cast<int>(*antenna_delay),
                                                                static_cast<int>(min_error));
                }
            }
            logger.log(buffer);
        }
    }
    distance = 0;
    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    bool got_poll = false;
    uint8_t n_tries = 0;
    while ((!got_poll) & (n_tries < 3)) {
        n_tries++;

        if (read_message() != 0) {
            /* Activate reception immediately. */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }

        if (rx_buffer[MSG_TYPE_IND] != 0x21) {
            continue;
        }
        got_poll = true;
    }

    if (!got_poll) {
        update_status(ModuleState::MODULE_IDLE);
        return -1;
    }

    rx_buffer[ALL_MSG_SN_IDX] = 0;

    /* Check that the frame is a poll sent by which anchor.
        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    anchor_id = rx_buffer[SOURCE_ID_IND];
    rx_poll_msg[SOURCE_ID_IND] = anchor_id;
    tx_resp_msg[DEST_ID_IND] = anchor_id;
    rx_final_msg[SOURCE_ID_IND] = anchor_id;
    if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) != 0) {
        return -1;
    }
    memset(&rx_buffer, 0, sizeof(rx_buffer));
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
        update_status(ModuleState::MODULE_ERROR);
        return -1;
    }

    /* Increment frame sequence number after transmission of the response message (modulo 256). */
    frame_seq_nb++;

    /* Try to read the final message 10 times */
    n_tries = 0;
    bool got_final = false;
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    while ((!got_final) & (n_tries < 5)) {
        n_tries++;
        if (read_message() != 0) {
            /* Activate reception immediately. */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        if (rx_buffer[MSG_TYPE_IND] != 0x23) {
            continue;
        }
        if ((rx_buffer[DEST_ID_IND] != ID) || (rx_buffer[SOURCE_ID_IND] != anchor_id)) {
            continue;
        }
        got_final = true;
    }

    if (!got_final) {
        update_status(ModuleState::MODULE_ERROR);
        return -1;
    }

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
    poll_rx_ts_32 = static_cast<uint32_t>(poll_rx_ts);
    resp_tx_ts_32 = static_cast<uint32_t>(resp_tx_ts);
    final_rx_ts_32 = static_cast<uint32_t>(final_rx_ts);
    Ra = static_cast<double>(resp_rx_ts - poll_tx_ts);
    Rb = static_cast<double>(final_rx_ts_32 - resp_tx_ts_32);
    Da = static_cast<double>(resp_tx_ts_32 - poll_rx_ts_32);
    Db = static_cast<double>(final_tx_ts - resp_rx_ts);
    tof_dtu = static_cast<int64>((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    if (distance < 0) {
        update_status(ModuleState::MODULE_IDLE);
        return -1;
    }

    /* Display computed distance on LCD. */
    uint32_t dist = static_cast<uint32_t>(distance * 1000);
    log_data[0] = anchor_id;
    log_data[1] =  dist & 0xFF;
    log_data[2] = (dist >> 8) & 0xFF;
    log_data[3] = (dist >> 16) & 0xFF;
    log_data[4] = (dist >> 24) & 0xFF;
    log_data[5] = 0xFF;
    log_data[6] = 0xFF;
    log_data[7] = 0xFF;
    log_data[8] = 0;
    logger.log(log_data, 9);
    last_success_time = HAL_GetTick();
    update_status(ModuleState::MODULE_OPERATIONAL);

    return 0;
}

/**
 * NOTES:
 * 
 * 9. Timestamps and delayed transmission time are both expressed in device time units so
 *    we just have to add the desired response delay to poll RX timestamp to get response
 *    transmission time. The delayed transmission time resolution is 512 device time units
 *    which means that the lower 9 bits of the obtained value must be zeroed.
 *    This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero
 *    lower 8 bits.
 */
