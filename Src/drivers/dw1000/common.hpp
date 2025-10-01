/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include "../dw1000/dw1000.hpp"
#include "../dw1000/parameters.hpp"
#include "../dw1000/configs.hpp"
#include "../uart_logger/logger.hpp"
#include "peripheral/flash/flash.h"

/* Define the global dw1000 object */
DW1000 dw1000;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define UUS_TO_DWT_TIME 65536
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

// /* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
// #define TX_ANT_DLY 16436
// #define RX_ANT_DLY 16436
const char FLASH_NAME_STR[] = "DCW";

#define CALIBRATION_DST    8140  // mm
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
    DWT_PLEN_2048,   /* Preamble length. Used in TX only. */
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
uint32_t DW1000::ant_dly = 0;
int DW1000::calibration_error = 0;
bool DW1000::calibration = false;
int DW1000::best_calibration_error = INT32_MAX;
uint16_t DW1000::calibration_frames_counter = 0;
uint8_t DW1000::calibration_step = 10;

uint8_t DW1000::rx_buffer[RX_BUF_LEN] = {};

int save_reference_values(ref_values reference_values) {
    flashLock();
    auto res = flashWrite(reinterpret_cast<uint8_t*>(&reference_values.PGdly),
                get_param_offset(ParametersNames::REFERENCE_PGdly), PARAMS_SIZE);
    res += flashWrite(reinterpret_cast<uint8_t*>(&reference_values.power),
                get_param_offset(ParametersNames::REFERENCE_POWER), PARAMS_SIZE);
    res +=flashWrite(reinterpret_cast<uint8_t*>(&reference_values.raw_temperature),
                get_param_offset(ParametersNames::REFERENCE_TEMP), PARAMS_SIZE);
    res += flashWrite(reinterpret_cast<uint8_t*>(&reference_values.count),
                get_param_offset(ParametersNames::REFERENCE_PG_COUNT), PARAMS_SIZE);
    flashUnlock();
    return res;
}

ref_values load_reference_values() {
    flashLock();
    ref_values ref;
    auto res = flashRead(reinterpret_cast<uint8_t*>(&ref.PGdly),
                get_param_offset(ParametersNames::REFERENCE_PGdly), PARAMS_SIZE);
    res += flashRead(reinterpret_cast<uint8_t*>(&ref.power),
                get_param_offset(ParametersNames::REFERENCE_POWER), PARAMS_SIZE);
    res +=flashRead(reinterpret_cast<uint8_t*>(&ref.raw_temperature),
                get_param_offset(ParametersNames::REFERENCE_TEMP), PARAMS_SIZE);
    res += flashRead(reinterpret_cast<uint8_t*>(&ref.count),
                get_param_offset(ParametersNames::REFERENCE_PG_COUNT), PARAMS_SIZE);
    flashUnlock();
    return ref;
}

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
    dwt_setrxantennadelay(ant_dly);
    dwt_settxantennadelay(ant_dly);
    return 0;
}

void DW1000::calibrate() {
    port_set_dw1000_slowrate();

    /* Reset and initialise DW1000. See NOTE 3 below. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    if (dwt_initialise(DWT_READ_OTP_TMP) == DWT_ERROR) return;
    auto txconfig = get_tx_config(dw_config.chan, dw_config.prf);
    dwt_configuretxrf(&txconfig);

    load_params();
    if (reference_values.raw_temperature == 0) {
        flashLock();
        flashWrite(reinterpret_cast<const uint8_t*>(FLASH_NAME_STR),
                    get_param_offset(ParametersNames::FLASH_NAME), PARAMS_SIZE);
        flashUnlock();
        /* Read DW1000 IC temperature for temperature compensation procedure. See NOTE 3 */
        reference_values.raw_temperature = (dwt_readtempvbat(1) & 0xFF00) >> 8;
        reference_values.PGdly = txconfig.PGdly;
        reference_values.power = txconfig.power;
        reference_values.count = dwt_calcpgcount(txconfig.PGdly);
        auto res = save_reference_values(reference_values);
        if (res < 4 * PARAMS_SIZE) {
            logger.log("ERROR");
            return;
        }
        dwt_softreset();
    }
    auto temp = (dwt_readtempvbat(1) & 0xFF00) >> 8;

    /* Compensate bandwidth and power settings for temperature */
    txconfig.PGdly = dwt_calcbandwidthtempadj(reference_values.count);
    txconfig.power = dwt_calcpowertempadj(dw_config.chan, reference_values.power,
            static_cast<int>(temp - reference_values.raw_temperature));

    /* Configure the TX frontend with the adjusted settings */
    dwt_configuretxrf(&txconfig);

    /* Software reset of the DW1000 to deactivate continuous frame mode and go back to default state. Initialisation and configuration should be run
     * again if one wants to get the DW1000 back to normal operation. */
    dwt_softreset();
}

void DW1000::calibrate_antenna_delay(uint32_t dist) {
    calibration_error += dist - CALIBRATION_DST;
    calibration_frames_counter++;

    if (calibration_frames_counter < 1000) {
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        return;
    }
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    calibration_error /= calibration_frames_counter;
    if (calibration_error < best_calibration_error) {
        logger.log("NEW BEST");
        best_calibration_error = calibration_error;
        flashWrite(reinterpret_cast<uint8_t*>(&best_calibration_error),
                get_param_offset(ParametersNames::BEST_CALIBRATION_ERROR), PARAMS_SIZE);
        flashWrite(reinterpret_cast<uint8_t*>(&ant_dly),
                get_param_offset(ParametersNames::ANTENNA_DELAY), PARAMS_SIZE);
    }
    if (ant_dly >= 100000) {
        calibration = false;
        logger.log("CALIBRATION FINISHED");
        return;
    }
    ant_dly += calibration_step;
    common_reset();
}

void DW1000::set_deafult_params() {
    ant_dly = 16436;
    best_calibration_error = INT32_MAX;
    reference_values.PGdly = 0;
    reference_values.power = 0;
    reference_values.raw_temperature = 0;
    reference_values.count = 0;
    save_reference_values(reference_values);
    flashLock();
    flashWrite(reinterpret_cast<const uint8_t*>(FLASH_NAME_STR),
                get_param_offset(ParametersNames::FLASH_NAME), PARAMS_SIZE);
    flashWrite(reinterpret_cast<uint8_t*>(&ant_dly),
                get_param_offset(ParametersNames::ANTENNA_DELAY), PARAMS_SIZE);
    flashWrite(reinterpret_cast<uint8_t*>(&best_calibration_error),
                get_param_offset(ParametersNames::BEST_CALIBRATION_ERROR), PARAMS_SIZE);
    flashUnlock();
}

void DW1000::load_params() {
    char param_name[4];
    int res = flashRead(reinterpret_cast<uint8_t*>(param_name), 0, 4);
    if (res != 4) {
        set_deafult_params();
        return;
    }

    if (strcmp(param_name, FLASH_NAME_STR) == 0) {
        /* Read antenna delay */
        res = flashRead(reinterpret_cast<uint8_t*>(&ant_dly),
        get_param_offset(ParametersNames::ANTENNA_DELAY), PARAMS_SIZE);
        if (res != 4) {
            set_deafult_params();
            return;
        }
        reference_values = load_reference_values();
        return;
    }

    if (res != 4) {
        set_deafult_params();
        return;
    }
    res = flashRead(reinterpret_cast<uint8_t*>(&best_calibration_error),
                get_param_offset(ParametersNames::BEST_CALIBRATION_ERROR), PARAMS_SIZE);
    if (res != 4) {
        best_calibration_error = INT32_MAX;
    }
}
