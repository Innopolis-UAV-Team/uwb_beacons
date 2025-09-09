/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "common.hpp"
#include "../uart_logger/logger.hpp"
#include "stm32f1xx_hal.h"
#include <cstdio>

/* Static variable definitions */
uint32_t DW1000::status_reg = 0;
uint8_t DW1000::frame_seq_nb = 0;
uint8_t DW1000::rx_buffer[RX_BUF_LEN] = {};
uint32_t tx_delay_ms = 0;

/* Interrupt state management variables */
volatile bool DW1000::tx_complete = false;
volatile bool DW1000::rx_complete = false;
volatile bool DW1000::rx_timeout = false;
volatile bool DW1000::rx_error = false;
volatile uint32_t DW1000::last_event_time = 0;

uint8_t log_data[9] = {};
/* State machine variables */
volatile DW1000::RangingState DW1000::current_state = DW1000::RangingState::IDLE;
volatile uint32_t DW1000::state_start_time = 0;
volatile uint8_t DW1000::current_anchor_id = 0;

/* Ranging data storage */
DW1000::RangingData DW1000::ranging_data = {0, 0, 0, 0, 0, 0, 0, false};

/* Global message definitions */
uint8_t poll_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x21, 0, 0};
uint8_t resp_msg[15] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
uint8_t final_msg[24] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x23, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Default inter-frame delay period, in milliseconds. */
#define DFLT_TX_DELAY_MS 10
/* Inter-frame delay period in case of RX timeout, in milliseconds.
 * In case of RX timeout, assume the receiver is not present and lower the rate of blink transmission. */
#define RX_TO_TX_DELAY_MS 30
/* Inter-frame delay period in case of RX error, in milliseconds.
 * In case of RX error, assume the receiver is present but its response has not been received for any reason and retry blink transmission immediately. */
#define RX_ERR_TX_DELAY_MS 0
/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */

/* This is the delay from the end of the frame transmission to the enable of the receiver,
as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define RESP_RX_TIMEOUT_UUS 2700
#define PRE_TIMEOUT 8

/* Define the global dw1000 object */
DW1000 dw1000;

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
dwt_config_t dw_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in TX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


int DW1000::common_reset() {
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        logger.log("INIT FAILED\n");
        return -1;
    }
    logger.log("INIT SUCCESS\n");
    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&dw_config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Setup interrupt-based operation */
    setup_interrupts();

    return 0;
}

void DW1000::setup_interrupts() {
    /* Install DW1000 IRQ handler. */
    port_set_deca_isr(dwt_isr);

    /* Register RX callbacks. */
    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::rx_ok_cb(const dwt_cb_data_t *cb_data) {
    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    /* Set event flags */
    rx_complete = true;
    rx_timeout = false;
    rx_error = false;
    last_event_time = HAL_GetTick();
    /* Set corresponding inter-frame delay.
     * For WAITING_RESPONSE, we must schedule the final frame immediately. */
    if (current_state == RangingState::WAITING_RESPONSE) {
        tx_delay_ms = 0;
    } else {
        tx_delay_ms = DFLT_TX_DELAY_MS;
    }

    /* Parse the received frame and handle state transitions */
    parse_received_frame();

    /* Handle state transitions based on current state and received message */
    switch (current_state) {
        case RangingState::WAITING_RESPONSE:
            /* Response received, now send final message */
            change_state(RangingState::SENDING_FINAL);
            break;

        case RangingState::WAITING_POLL:
            /* Poll received, now send response */
            change_state(RangingState::SENDING_RESPONSE);
            break;

        case RangingState::WAITING_FINAL:
            /* Final message received, process result */
            change_state(RangingState::PROCESSING_RESULT);
            break;

        default:
            break;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::rx_to_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data; /* Unused parameter */

    /* Set event flags */
    rx_timeout = true;
    rx_complete = false;
    rx_error = false;
    last_event_time = HAL_GetTick();
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_TO_TX_DELAY_MS;

    /* Handle timeout - restart ranging cycle */
    start_next_ranging_cycle();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::rx_err_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data; /* Unused parameter */

    /* Set event flags */
    rx_error = true;
    rx_complete = false;
    rx_timeout = false;
    last_event_time = HAL_GetTick();
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_ERR_TX_DELAY_MS;
    /* Handle error - restart ranging cycle */
    start_next_ranging_cycle();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void DW1000::tx_conf_cb(const dwt_cb_data_t *cb_data) {
    (void)cb_data; /* Unused parameter */

    /* Set event flags */
    tx_complete = true;
    last_event_time = HAL_GetTick();

    /* Handle state transitions based on current state */
    switch (current_state) {
        case RangingState::SENDING_POLL:
            /* Poll message sent, now wait for response */
            change_state(RangingState::WAITING_RESPONSE);
            break;

        case RangingState::SENDING_RESPONSE:
            /* Response sent, now wait for final message */
            /* Capture actual response TX timestamp for DS-TWR calculation */
            ranging_data.resp_tx_ts = get_tx_timestamp_u64();
            change_state(RangingState::WAITING_FINAL);
            break;

        case RangingState::SENDING_FINAL:
            /* Final message sent, process result */
            change_state(RangingState::PROCESSING_RESULT);
            break;

        default:
            break;
    }
}

/* Helper function implementations */
void DW1000::change_state(RangingState new_state) {
    current_state = new_state;
    state_start_time = HAL_GetTick();
}

void DW1000::parse_received_frame() {
    /* Parse the received frame to extract anchor ID and validate message type */
    if (rx_complete && rx_buffer[0] == 0x41 && rx_buffer[1] == 0x88) {
        /* Extract anchor ID from the frame */
        current_anchor_id = rx_buffer[8];  // Assuming anchor ID is at position 8

        /* Validate message type and extract timestamps */
        if (rx_buffer[9] == 0x21) {
            /* Poll message received */
            ranging_data.poll_rx_ts = get_rx_timestamp_u64();
            ranging_data.anchor_id = current_anchor_id;
        } else if (rx_buffer[9] == 0x10) {
            /* Response message received */
            ranging_data.resp_rx_ts = get_rx_timestamp_u64();
        } else if (rx_buffer[9] == 0x23) {
            /* Final message received */
            ranging_data.final_rx_ts = get_rx_timestamp_u64();
            /* Extract embedded timestamps from final message */
            uint32_t poll_tx_ts_32 = 0;
            uint32_t resp_rx_ts_32 = 0;
            uint32_t final_tx_ts_32 = 0;
            final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_32);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);
            /* Store as 64-bit with upper 32 bits cleared to avoid stale data */
            ranging_data.poll_tx_ts = static_cast<uint64_t>(poll_tx_ts_32);
            ranging_data.resp_rx_ts = static_cast<uint64_t>(resp_rx_ts_32);
            ranging_data.final_tx_ts = static_cast<uint64_t>(final_tx_ts_32);
            ranging_data.data_valid = true;
        }
    }
}

void DW1000::process_ranging_result() {
    if (ranging_data.data_valid) {
        /* Calculate time of flight and distance
        Compute time of flight.
        32-bit subtractions give correct answers even if clock has wrapped. */
        uint32_t poll_rx_ts_32 = static_cast<uint32_t>(ranging_data.poll_rx_ts);
        uint32_t resp_tx_ts_32 = static_cast<uint32_t>(ranging_data.resp_tx_ts);
        uint32_t final_rx_ts_32 = static_cast<uint32_t>(ranging_data.final_rx_ts);

        /* Use 32-bit arithmetic for wrapped differences as per Decawave reference */
        double Ra = static_cast<double>(static_cast<uint32_t>(ranging_data.resp_rx_ts) - static_cast<uint32_t>(ranging_data.poll_tx_ts));
        double Rb = static_cast<double>(final_rx_ts_32 - resp_tx_ts_32);
        double Da = static_cast<double>(static_cast<uint32_t>(ranging_data.final_tx_ts) - static_cast<uint32_t>(ranging_data.resp_rx_ts));
        double Db = static_cast<double>(resp_tx_ts_32 - poll_rx_ts_32);

        int64_t tof_dtu = static_cast<int64_t>((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;
        if (distance < 0) {
            distance = 0;
        }

        /* Log the result */
        uint32_t dist = static_cast<uint32_t>(static_cast<int>(distance * 1000));
        log_data[0] = ranging_data.anchor_id;
        log_data[1] = dist & 0xFF;
        log_data[2] = (dist >> 8) & 0xFF;
        log_data[3] = (dist >> 16) & 0xFF;
        log_data[4] = (dist >> 24) & 0xFF;
        log_data[5] = 0xFF;
        log_data[6] = 0xFF;
        log_data[7] = 0xFF;
        log_data[8] = 0;
        logger.log(log_data, 9);

        ranging_data.data_valid = false;
    }

    /* Start next ranging cycle */
    start_next_ranging_cycle();
}

void DW1000::start_next_ranging_cycle() {
    /* Reset state and start new ranging cycle after delay */
    change_state(RangingState::IDLE);
    /* The spin() function will handle the timing and start the next cycle */
}

bool DW1000::is_timeout_elapsed(uint32_t timeout_ms) {
    return (HAL_GetTick() - state_start_time) >= timeout_ms;
}

/* Helper function implementations */
uint64_t get_tx_timestamp_u64(void) {
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

uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8_t) ts;
        ts >>= 8;
    }
}
