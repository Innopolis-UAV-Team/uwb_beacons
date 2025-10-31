/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/
#ifndef SRC_DRIVERS_DW1000_COMMON_HPP_
#define SRC_DRIVERS_DW1000_COMMON_HPP_

#include "dw1000.hpp"
#include "../uart_logger/logger.hpp"

/* Define the global dw1000 object */
extern DW1000 dw1000;
#define SOURCE_ID_IND           8
#define DEST_ID_IND             6
#define MSG_TYPE_IND 9

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
#define RNG_DELAY_MS 100

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
extern dwt_config_t dw_config;

extern uint8_t poll_msg[12];
extern uint8_t resp_msg[15];
extern uint8_t final_msg[24];

/* Static member definitions - these will be defined in dw1000.cpp */

/* Declaration of static functions. */
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

#endif  // SRC_DRIVERS_DW1000_COMMON_HPP_
