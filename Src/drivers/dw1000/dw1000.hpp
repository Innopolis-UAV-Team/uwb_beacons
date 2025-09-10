/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_DRIVERS_DW1000_DW1000_HPP_
#define SRC_DRIVERS_DW1000_DW1000_HPP_

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"
#include <map>

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
extern dwt_config_t config;

class DW1000 {
 public:
    int init();
    void spin();

 private:
    enum RangingState {
        IDLE,
        SENDING_POLL,
        WAITING_RESPONSE,
        SENDING_RESPONSE,
        PROCESSING_RESPONSE,
        WAITING_FINAL,
        SENDING_FINAL,
        PROCESSING_RESULT,
    };
    struct RangingData {
        uint64_t poll_tx_ts;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts;
        uint64_t resp_rx_ts;
        uint64_t final_rx_ts;
        uint64_t final_tx_ts;
        uint8_t anchor_id;
        RangingState state;
        uint8 frame_seq_nb;
        bool data_valid;
    };

    static std::map<uint8_t, RangingData> ranging_data;
    // static RangingState current_state;
    static uint32_t state_start_time;
    static uint8_t poll_msg[12];
    static uint8_t resp_msg[15];
    static uint8_t final_msg[24];
    // static bool message_sent;
    /*
    * @fn rx_ok_cb()
    *
    * @brief Callback to process RX good frame events
    *
    * @param  cb_data  callback data
    *
    * @return  none
    */
    static void rx_complete_cb(const dwt_cb_data_t *cb_data);
    /*
    * @fn rx_timeout_cb()
    *
    * @brief Callback to process RX timeout events
    *
    * @param  cb_data  callback data
    *
    * @return  none
    */
    static void rx_timeout_cb(const dwt_cb_data_t *cb_data);
    /*
    * @fn rx_error_cb()
    *
    * @brief Callback to process RX error events
    *
    * @param  cb_data  callback data
    *
    * @return  none
    */
    static void rx_error_cb(const dwt_cb_data_t *cb_data);
    /*
    * @fn tx_complete_cb()
    *
    * @brief Callback to process TX confirmation events
    *
    * @param  cb_data  callback data
    *
    * @return  none
    */
    static void tx_complete_cb(const dwt_cb_data_t *cb_data);
};

extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
