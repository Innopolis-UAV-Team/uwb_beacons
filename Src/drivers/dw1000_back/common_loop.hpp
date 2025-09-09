/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "../uart_logger/logger.hpp"

/* Define the global dw1000 object */
DW1000 dw1000;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define UUS_TO_DWT_TIME 65536
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define RNG_DELAY_MS 10

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
dwt_config_t dw_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

uint8_t poll_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x21, 0, 0};
uint8_t resp_msg[15] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
uint8_t final_msg[24] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00, 0x23, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint32_t DW1000::status_reg = 0;
uint8_t DW1000::frame_seq_nb = 0;
// uint64_t DW1000::poll_tx_ts = {};  // tx|rx changed
// uint64_t DW1000::resp_tx_ts = {};
// uint64_t DW1000::final_tx_ts = {};

// uint64_t DW1000::poll_rx_ts = {};  // tx|rx changed
// uint64_t DW1000::resp_rx_ts = {};
// uint64_t DW1000::final_rx_ts = {};
uint8_t DW1000::rx_buffer[RX_BUF_LEN] = {};


/* Declaration of static functions. */
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

int DW1000::common_reset() {
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        logger.log("INIT FAILED");
        return -1;
    }
    logger.log("INIT SUCCESS");
    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&dw_config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    return 0;
}
