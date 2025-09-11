/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include <stdio.h>
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include <cstdio>

/* Index to access the sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#ifndef ANCHOR_ID
#define ANCHOR_ID 0x0
#endif

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 50

/* Default inter-frame delay period, in milliseconds. */
#define DFLT_TX_DELAY_MS 10
/* Inter-frame delay period in case of RX timeout, in milliseconds.
 * In case of RX timeout, assume the receiver is not present and lower the rate of blink transmission. */
#define RX_TO_TX_DELAY_MS 10
/* Inter-frame delay period in case of RX error, in milliseconds.
 * In case of RX error, assume the receiver is present but its response has not been received for any reason and retry blink transmission immediately. */
#define RX_ERR_TX_DELAY_MS 0
/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */
static uint32 tx_delay_ms = 0;

/* Buffer to store received frame. See NOTE 4 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Frames used in the ranging process. See NOTE 2 below. */
const uint8_t RX_ANCHOR_ID_IND = 8;
const uint8_t TX_ANCHOR_ID_IND = 6;

uint8_t log_data[9] = {0};
// static uint64_t poll_rx_ts;  // tx|rx changed, partially transfered
// static uint64_t resp_tx_ts;
// static uint64_t final_rx_ts;
static uint8_t tx_anchor_id = 0;
volatile static uint32_t last_transmission_ms = 0;

/**
 * Application entry point.
 */
int DW1000::init(void) {
    /* Install DW1000 IRQ handler. */
    port_set_deca_isr(dwt_isr);

    /* Reset and initialise DW1000. See NOTE 5 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        return -1;
    }

    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Register RX call-back. */
    dwt_setcallbacks(&tx_complete_cb, &rx_complete_cb, &rx_timeout_cb, &rx_error_cb);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

    /* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);
    // /* Set response frame timeout. */
    // dwt_setrxtimeout(RX_RESP_TO_UUS);
    /* Clear reception timeout to start next ranging process. */
    dwt_setrxtimeout(0);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return 0;
}

void DW1000::spin_router_for_one_anchor(uint8_t anchor_id) {
    RangingData* data = getDataEntryById(anchor_id);

    if (HAL_GetTick() - data->start_state_time > 1000) {
        data->state = RangingState::IDLE;
    }

    if (data->state == RangingState::PROCESSING_POLL) {
        if (HAL_GetTick() - last_transmission_ms < (tx_delay_ms)) {
            return;
        }
        uint32_t resp_tx_time;
        resp_tx_time = (data->poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);
        /* Set expected delay and timeout for final message reception. */
        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
        resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        /* Zero offset in TX buffer. */
        dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
        /* Zero offset in TX buffer, ranging. */
        dwt_writetxfctrl(sizeof(resp_msg), 0, 1);
        dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
        data->state = RangingState::SENDING_RESPONSE;
        data->start_state_time = HAL_GetTick();
    }
    if (data->state == RangingState::PROCESSING_RESULT) {
        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
        double Ra, Rb, Da, Db, tof, distance;
        int64_t tof_dtu;
        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
        data->state = RangingState::IDLE;
        data->start_state_time = HAL_GetTick();
        poll_rx_ts_32 = static_cast<uint32_t>(data->poll_rx_ts);
        resp_tx_ts_32 = static_cast<uint32_t>(data->resp_tx_ts);
        final_rx_ts_32 = static_cast<uint32_t>(data->final_rx_ts);
        Ra = static_cast<double>(resp_rx_ts - poll_tx_ts);
        Rb = static_cast<double>(final_rx_ts_32 - resp_tx_ts_32);
        Da = static_cast<double>(final_tx_ts - resp_rx_ts);
        Db = static_cast<double>(resp_tx_ts_32 - poll_rx_ts_32);
        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        tof = tof_dtu * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;
        if (distance < 0) {
            return;
        }

        /*Send computed distance to logger*/
        auto dist = static_cast<uint32_t>(distance * 1000);
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
        data->state = RangingState::IDLE;
        return;
    }

    data->state = RangingState::IDLE;
    return;
}

void DW1000::spin() {
    if (data_array_entry_count == 0) {
        if (HAL_GetTick() - last_transmission_ms > 1000) {
            ack_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
            dwt_writetxdata(sizeof(ack_msg), ack_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(ack_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            /* Start transmission, indicating that a response is expected so that reception
            is enabled immediately after the frame is sent. */
            dwt_starttx(DWT_START_TX_IMMEDIATE);
            last_transmission_ms = HAL_GetTick();
        }
        return;
    }
    for (uint8_t i = 0; i < data_array_entry_count; i++) {
        spin_router_for_one_anchor(data_array[i].id);
    }
}

void DW1000::rx_complete_cb(const dwt_cb_data_t *cb_data) {
    int i;
    /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
     * buffer. */
    for (i = 0; i < FRAME_LEN_MAX; i++) {
        rx_buffer[i] = 0;
    }

    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= FRAME_LEN_MAX) {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    /* Set corresponding inter-frame delay. */
    tx_delay_ms = DFLT_TX_DELAY_MS;
    /* Check message type */
    RangingData* data = nullptr;
    uint8_t sender_id = 0;
    switch (rx_buffer[MESSAGE_TYPE_IDX]) {
    case RESP_MSG_SPECIAL_ID:
        return;
    case ACKN_MSG_SPECIAL_ID:
        return;
    case POLL_MSG_SPECIAL_ID:
        sender_id = rx_buffer[POLL_FIN_ID_IND];
        data = getDataEntryById(sender_id);
        if (data->state == RangingState::IDLE || data->state == RangingState::SENDING_ACK) {
            data->poll_rx_ts = dwt_readrxtimestamplo32();
            data->state = RangingState::PROCESSING_POLL;
            data->start_state_time = HAL_GetTick();
        }
        return;
    case FINL_MSG_SPECIAL_ID:
        sender_id = rx_buffer[POLL_FIN_ID_IND];
        data = getDataEntryById(sender_id);

        if (data->state == RangingState::WAITING_FINAL) {
            data->final_rx_ts = dwt_readrxtimestamplo32();
            data->state = RangingState::PROCESSING_RESULT;
            data->start_state_time = HAL_GetTick();
        }
        return;
    default:
        sender_id = rx_buffer[POLL_FIN_ID_IND];
        data = getDataEntryById(sender_id);

        break;
    }
}

void DW1000::rx_timeout_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_TO_TX_DELAY_MS;
}

void DW1000::rx_error_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_ERR_TX_DELAY_MS;
}

void DW1000::tx_complete_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    if (data_array_entry_count == 0) {
        last_transmission_ms = HAL_GetTick();
        return;
    }
    RangingData* data = getValueById(tx_anchor_id);
    switch (data->state) {
        case RangingState::SENDING_RESPONSE:
            data->poll_tx_ts = get_tx_timestamp_u64();
            data->state = RangingState::WAITING_FINAL;
            data->start_state_time = HAL_GetTick();
            break;
        case RangingState::SENDING_ACK:
            data->state = RangingState::IDLE;
            data->start_state_time = HAL_GetTick();
            break;
        default:
            data->state = RangingState::IDLE;
            break;
    }
    frame_seq_nb++;
    state_start_time = HAL_GetTick();
    last_transmission_ms = HAL_GetTick();
    /* Reset the TX delay and event signalling mechanism ready to await the next event. */
    tx_delay_ms =  0;
}
