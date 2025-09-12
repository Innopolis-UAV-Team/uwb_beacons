/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "dw1000.hpp"
#include "common.hpp"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
dwt_config_t config = {
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

// ref_values DW1000::reference = {
//         0xC0,           /* PG_DELAY */
//         0x25456585,     /* Power */
//         0x81,          /* Temp */
//         0x369           /* PG_COUNT */
// };

DW1000 dw1000;

// void DW1000::calibrate() {
//     dwt_txconfig_t txconfig;
//     int temp;

//     /* Read DW1000 IC temperature for temperature compensation procedure. See NOTE 4 */
//     temp = (dwt_readtempvbat(1) & 0xFF00) >> 8;

//     /* Compensate bandwidth and power settings for temperature */
//     txconfig.PGdly = dwt_calcbandwidthtempadj(reference.count);
//     txconfig.power = dwt_calcpowertempadj(config.chan, reference.power, (int) (temp - reference.raw_temperature));

//     /* Configure the TX frontend with the adjusted settings */
//     dwt_configuretxrf(&txconfig);
// }

TxConfig DW1000::tx_config = {0, 0};
uint32_t DW1000::state_start_time = 0;
DataEntry DW1000::data_array[MAX_ENTRIES] = {};
uint8_t DW1000::frame_seq_nb = 0;
uint8_t DW1000::data_array_entry_count = 0;
uint8_t DW1000::poll_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00,
                                POLL_MSG_SPECIAL_ID, 0, 0};
uint8_t DW1000::resp_msg[15] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 0x00, 'W', 'A',
                                RESP_MSG_SPECIAL_ID, 0x02, 0, 0, 0, 0};
uint8_t DW1000::final_msg[24] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00,
                                FINL_MSG_SPECIAL_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t DW1000::ack_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 0x00,
                                ACKN_MSG_SPECIAL_ID, 0, 0};
__weak void DW1000::spin_router_for_one_anchor(uint8_t anchor_id) {
    (void)anchor_id;
}
