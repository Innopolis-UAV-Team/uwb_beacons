#pragma once
#include "deca_device_api.h"

struct tx_struct {
    uint8_t PG_DELAY;
    uint32_t tx_pwr[2];  // 16M and 64M
};

struct ref_values {
    uint8 PGdly;
    uint32 power;
    int raw_temperature;
    // the DW IC raw register value, which needs conversion if you want a °C value
    uint16 count;
};

static const uint8_t NUM_CH = 7;

const tx_struct tx_spectrumconfig[NUM_CH] = {
    { 0xc9, {0x75757575, 0x67676767} },  // Channel 1
    { 0xC2, {0x75757575, 0x67676767} },  // Channel 2
    { 0xC5, {0x6F6F6F6F, 0x8B8B8B8B} },  // Channel 3
    { 0x95, {0x5F5F5F5F, 0x9A9A9A9A} },  // Channel 4
    { 0xC0, {0x48484848, 0x85858585} },  // Channel 5
    { 0x00, {0x00,       0x00} },        // Channel 6 (not supported)
    { 0x93, {0x92929292, 0xD1D1D1D1} },  // Channel 7
};

const tx_struct tx_spectrumconfig_smart_power[NUM_CH] = {
    { 0xc9, {0x15355575, 0x07274767} },  // Channel 1
    { 0xC2, {0x15355575, 0x07274767} },  // Channel 2
    { 0xC5, {0x0F2F4F6F, 0x2B4B6B8B} },  // Channel 3
    { 0x95, {0x1F1F3F5F, 0x3A5A7A9A} },  // Channel 4
    { 0xC0, {0x0E082848, 0x25456585} },  // Channel 5
    { 0x00, {0x00,       0x00} },        // Channel 6 (not supported)
    { 0x93, {0x32527292, 0x5171B1D1} },  // Channel 7
};

/*
* Get the TX configuration for the given channel and PRF from reference measurements (see README.md)
* If smart_power is true, use the smart power compensation
* @param chan Channel number (1-7). 6 is not supported.
* @param prf pulse repetition frequency (DWT_PRF_16M or DWT_PRF_64M)
* @param smart_power Use smart power compensation, default is false
* @return dwt_txconfig_t
*/
dwt_txconfig_t inline get_tx_config(uint8_t chan, uint8_t prf, bool smart_power = false) {
    tx_struct tx_config = smart_power ?
                            tx_spectrumconfig_smart_power[chan - 1] :
                            tx_spectrumconfig[chan - 1];
    dwt_txconfig_t txconfig = { .PGdly = tx_config.PG_DELAY,
                                .power = tx_config.tx_pwr[prf - DWT_PRF_16M]};
    return txconfig;
}
