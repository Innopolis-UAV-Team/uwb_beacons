/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "common.hpp"

/* Index to access the sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#ifndef ANCHOR_ID
#define ANCHOR_ID 0x1
#endif

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 5000

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
static uint32_t last_transmission_ms = 0;
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
    final_msg[POLL_FIN_ID_IND]  = ANCHOR_ID;
    poll_msg[POLL_FIN_ID_IND]   = ANCHOR_ID;

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Register RX call-back. */
    dwt_setcallbacks(&tx_complete_cb, &rx_complete_cb, &rx_timeout_cb, &rx_error_cb);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

    /* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);
    RangingData data = {0, 0, 0, 0, 0, 0, ANCHOR_ID, IDLE, 0, 0, false};
    addDataEntry(ANCHOR_ID, data);
    return 0;
}

void DW1000::spin() {
    if (HAL_GetTick() - last_transmission_ms < (tx_delay_ms)) {
        return;
    }

    RangingData* data = getValueById(ANCHOR_ID);
    if (data == nullptr) {
        return;
    }
    if (data->state == RangingState::IDLE) {
        last_transmission_ms = HAL_GetTick();
        poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        dwt_writetxdata(sizeof(poll_msg), poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        /* Start transmission, indicating that a response is expected so that reception
        is enabled immediately after the frame is sent. */
        data->state = SENDING_POLL;
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        data->start_state_time = HAL_GetTick();
    }
    if (data->state == RangingState::PROCESSING_RESPONSE) {
        /* Write and send final message. See NOTE 8 below. */
        final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        /* Write all timestamps in the final message. See NOTE 11 below. */
        final_msg_set_ts(&final_msg[FINAL_MSG_POLL_TX_TS_IDX], data->poll_tx_ts);
        final_msg_set_ts(&final_msg[FINAL_MSG_RESP_RX_TS_IDX], data->resp_rx_ts);
        /* Compute final message transmission time. See NOTE 10 below. */
        uint32_t final_tx_time = (data->resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(final_tx_time);
        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        uint64_t final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        final_msg_set_ts(&final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
        dwt_writetxdata(sizeof(final_msg), final_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        dwt_starttx(DWT_START_TX_DELAYED);
        data->state = RangingState::SENDING_FINAL;
        data->start_state_time = HAL_GetTick();
    }
    if (HAL_GetTick() - data->start_state_time > 50) {
        data->state = RangingState::IDLE;
    }
}

void DW1000::rx_complete_cb(const dwt_cb_data_t *cb_data) {
    int i;
    uint32_t frame_len = 0;
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
    switch (rx_buffer[MESSAGE_TYPE_IDX]) {
    case RESP_MSG_SPECIAL_ID:
        data = getValueById(ANCHOR_ID);
        if (data->state == RangingState::WAITING_RESPONSE) {
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            uint8_t response_id = rx_buffer[RESPONSE_ID_IND];
            if (response_id != ANCHOR_ID) {
                return;
            }
            data->resp_rx_ts = get_rx_timestamp_u64();
            data->state = RangingState::PROCESSING_RESPONSE;
            data->start_state_time = HAL_GetTick();
        }
        return;
    case ACKN_MSG_SPECIAL_ID:
        data = getValueById(ANCHOR_ID);
        if (data == nullptr) {
            RangingData ranging_data = RangingData{0, 0, 0, 0, 0, 0,
                ANCHOR_ID, IDLE, 0, 0, false};
            addDataEntry(ANCHOR_ID, ranging_data);
            return;
        }
        data->state = RangingState::IDLE;
        break;
    default:
        uint8_t other_id = rx_buffer[POLL_FIN_ID_IND];
        data = getValueById(other_id);
        if (data == nullptr) {
            RangingData ranging_data = RangingData{0, 0, 0, 0, 0, 0,
                rx_buffer[ALL_MSG_SN_IDX], IDLE, 0, 0, false};
            addDataEntry(rx_buffer[ALL_MSG_SN_IDX], ranging_data);
            return;
        }
        break;
    }
}

void DW1000::rx_timeout_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_TO_TX_DELAY_MS;
    RangingData* data = getValueById(ANCHOR_ID);
    if (data == nullptr) {
        return;
    }
    data->state = RangingState::IDLE;
}

void DW1000::rx_error_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_ERR_TX_DELAY_MS;
    RangingData* data = getValueById(ANCHOR_ID);
    if (data == nullptr) {
        return;
    }
    data->state = RangingState::IDLE;
}

void DW1000::tx_complete_cb(const dwt_cb_data_t *cb_data) {
    (void)(cb_data);
    RangingData* data = getValueById(ANCHOR_ID);
    switch (data->state) {
        case RangingState::SENDING_POLL:
            data->poll_tx_ts = get_tx_timestamp_u64();
            data->state = RangingState::WAITING_RESPONSE;
            data->start_state_time = HAL_GetTick();
            break;
        case RangingState::SENDING_FINAL:
            data->state = RangingState::IDLE;
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

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW1000 unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW1000 during its manufacture. However there is no guarantee this will not conflict with someone else�s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW1000 User Manual.
 * 2. TX to RX delay can be set to 0 to activate reception immediately after transmission. But, on the responder side, it takes time to process the
 *    received frame and generate the response (this has been measured experimentally to be around 70 �s). Using an RX to TX delay slightly less than
 *    this minimum turn-around time allows the application to make the communication efficient while reducing power consumption by adjusting the time
 *    spent with the receiver activated.
 * 3. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive a complete frame sent by the "RX then send a response"
 *    example at the 110k data rate used (around 3 ms).
 * 4. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended frame
 *    length (up to 1023 bytes long) mode which is not used in this example.
 * 5. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
