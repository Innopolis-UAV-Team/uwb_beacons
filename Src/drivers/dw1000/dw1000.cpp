/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/
#include "dw1000.hpp"
#include "common.hpp"

/* Define the global dw1000 object */
DW1000 dw1000;

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
dwt_config_t dw_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_2048,   /* Preamble length. Used in TX only. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    // 4096+64
    (2048 + 1 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

uint8_t poll_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x21, 0, 0};
uint8_t resp_msg[15] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
uint8_t final_msg[24] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x23, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t got_best_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x18, 0, 0};

/* Static member definitions */
uint32_t DW1000::status_reg = 0;
uint8_t DW1000::frame_seq_nb = 0;
uint8_t DW1000::rx_buffer[RX_BUF_LEN] = {};

int8_t DW1000::read_message() {
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

    if (!(status_reg & SYS_STATUS_RXFCG)) {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        return -1;
    }

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
        return 0;
    }
    return -1;
}

int DW1000::common_reset() {
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    get_current_ant_delay();

    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        logger.log("INIT FAILED");
        return -1;
    }
    logger.log("IN OK");
    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&dw_config);
    dwt_configurefor64plen(dw_config.prf);

    if (!is_calibration) {
        *antenna_delay = TX_ANT_DLY;
    } else {
        if (((*antenna_delay) > 32872) & (min_error > 0)) {
            *antenna_delay = best_antenna_delay;
            is_calibration = false;
            logger.log("CAL DONE");
        }
    }
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(*antenna_delay);
    dwt_settxantennadelay(*antenna_delay);
    return 0;
}

/**
 * Estimate the transmission time of a frame in seconds
 * Author: tomlankhorst 2016 https://tomlankhorst.nl/estimating-decawave-dw1000-tx-time/
 * dwt_config_t   dwt_config  Configuration struct of the DW1000
 * uint16_t       framelength Framelength including the 2-Byte CRC
 * bool           only_rmarker Option to compute only the time to the RMARKER (SHR time)
 */
float dwt_estimate_tx_time(dwt_config_t dwt_config, uint16_t framelength, bool only_rmarker) {
    int32_t tx_time;
    size_t sym_timing_ind;
    uint16_t shr_len;

    const uint16_t DATA_BLOCK_SIZE  = 330;
    const uint16_t REED_SOLOM_BITS  = 48;

    // Symbol timing LUT
    const size_t SYM_TIM_16MHZ = 0;
    const size_t SYM_TIM_64MHZ = 9;
    const size_t SYM_TIM_110K  = 0;
    const size_t SYM_TIM_850K  = 3;
    const size_t SYM_TIM_6M8   = 6;
    const size_t SYM_TIM_SHR   = 0;
    const size_t SYM_TIM_PHR   = 1;
    const size_t SYM_TIM_DAT   = 2;

    const static uint16_t SYM_TIM_LUT[] = {
        // 16 Mhz PRF
        994, 8206, 8206,  // 0.11 Mbps
        994, 1026, 1026,  // 0.85 Mbps
        994, 1026, 129,   // 6.81 Mbps
        // 64 Mhz PRF
        1018, 8206, 8206, // 0.11 Mbps
        1018, 1026, 1026, // 0.85 Mbps
        1018, 1026, 129   // 6.81 Mbps
    };

    // Find the PHR
    switch( dwt_config.prf ) {
        case DWT_PRF_16M:  sym_timing_ind = SYM_TIM_16MHZ; break;
        case DWT_PRF_64M:  sym_timing_ind = SYM_TIM_64MHZ; break;
        default: return -1.0f; // Invalid PRF
    }

    // Find the preamble length
    switch( dwt_config.txPreambLength ) {
        case DWT_PLEN_64:    shr_len = 64;    break;
        case DWT_PLEN_128:  shr_len = 128;  break;
        case DWT_PLEN_256:  shr_len = 256;  break;
        case DWT_PLEN_512:  shr_len = 512;  break;
        case DWT_PLEN_1024: shr_len = 1024;  break;
        case DWT_PLEN_1536: shr_len = 1536;  break;
        case DWT_PLEN_2048: shr_len = 2048;  break;
        case DWT_PLEN_4096: shr_len = 4096;  break;
        default: return -1.0f; // Invalid preamble length
    }

    // Find the datarate
    switch( dwt_config.dataRate ) {
        case DWT_BR_110K:
            sym_timing_ind  += SYM_TIM_110K;
            shr_len         += 64;  // SFD 64 symbols
            break;
        case DWT_BR_850K:
            sym_timing_ind  += SYM_TIM_850K;
            shr_len         += 8;   // SFD 8 symbols
            break;
        case DWT_BR_6M8:
            sym_timing_ind  += SYM_TIM_6M8;
            shr_len         += 8;   // SFD 8 symbols
            break;
        default: return -1.0f;  // Invalid bitrate
    }

    // Add the SHR time
    tx_time   = shr_len * SYM_TIM_LUT[ sym_timing_ind + SYM_TIM_SHR ];

    // If not only RMARKER, calculate PHR and data
    if (!only_rmarker) {
        // Add the PHR time (21 bits)
        tx_time  += 21 * SYM_TIM_LUT[ sym_timing_ind + SYM_TIM_PHR ];

        // Bytes to bits
        framelength *= 8;

        // Add Reed-Solomon parity bits
        framelength += REED_SOLOM_BITS * (framelength + DATA_BLOCK_SIZE - 1) / DATA_BLOCK_SIZE;

        // Add the DAT time
        tx_time += framelength * SYM_TIM_LUT[ sym_timing_ind + SYM_TIM_DAT ];
    }

    // Return float seconds
    return (1.0e-9f) * tx_time;
}

void DW1000::get_current_ant_delay() {
    if (dw_config.prf == DWT_PRF_64M) {
        antenna_delay = &antenna_delays.tx_rx_prf_64;
        return;
    }
    antenna_delay = &antenna_delays.tx_rx_prf_16;
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
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8_t) ts;
        ts >>= 8;
    }
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
uint64_t get_rx_timestamp_u64(void) {
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

enum dwt_calibration_type: uint8_t {
    DWT_ANTENNA_RX_DELAY = 0,
    DWT_ANTENNA_TX_DELAY,
};

void DW1000::set_calibration(int id, uint16_t real_distance_mm) {
    seeked_id = id;
    real_distance = real_distance_mm;
    is_calibration = true;
    get_current_ant_delay();
    *antenna_delay = 10000;
    // antenna_delays = 1000;
    min_error = MAXFLOAT;
}
