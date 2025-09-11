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

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
extern dwt_config_t config;
const uint8_t POLL_FIN_ID_IND = 8;
const uint8_t RESPONSE_ID_IND = 6;

class DW1000 {
 public:
    int init();
    void spin();
    static const uint8_t MAX_ENTRIES = 20;

 private:
    enum RangingState {
        IDLE = 0,
        SENDING_POLL,
        PROCESSING_POLL,
        WAITING_RESPONSE,
        SENDING_RESPONSE,
        PROCESSING_RESPONSE,
        WAITING_FINAL,
        SENDING_FINAL,
        PROCESSING_RESULT,
        SENDING_ACK,
    };
    struct RangingData {
        uint32_t poll_tx_ts;
        uint64_t poll_rx_ts;
        uint64_t resp_tx_ts;
        uint32_t resp_rx_ts;
        uint64_t final_rx_ts;
        uint64_t final_tx_ts;
        uint8_t anchor_id;
        RangingState state;
        uint32_t start_state_time;
        bool data_valid;
    };
    typedef struct {
        int id;
        RangingData value;
    } DataEntry;

    struct TxConfig {
        uint8_t id;
        uint8_t msg_type;
    };
    static TxConfig tx_config;
    static uint8_t frame_seq_nb;
    static DataEntry data_array[MAX_ENTRIES];
    static uint8_t data_array_entry_count;
    /*
    * @fn getValueById()
    * @brief Get the value of the entry with the given ID
    * @param id ID of the entry
    * @return Pointer to the value of the entry with the given ID, or nullptr if not found
    */
    static inline RangingData* getValueById(uint8_t id) {
        for (int i = 0; i < data_array_entry_count; i++) {
            if (data_array[i].id == id) {
                return &data_array[i].value;  // Return the value if ID matches
            }
        }
        return nullptr;
    }
    static void addDataEntry(uint8_t id, RangingData data) {
        if (data_array_entry_count < MAX_ENTRIES) {
            data_array[data_array_entry_count].id = id;
            data_array[data_array_entry_count].value = data;
            data_array_entry_count++;
        }
    }

    static RangingData* getDataEntryById(uint8_t id) {
        RangingData * data = getValueById(id);
        if ((data == nullptr) | (data_array_entry_count == 0)) {
            RangingData ranging_data = RangingData{0, 0, 0, 0, 0, 0,
                                                    id, IDLE, 0, false};
            addDataEntry(id, ranging_data);
            data = getValueById(id);
        }
        return data;
    }

    // static RangingState current_state;
    static uint32_t state_start_time;

    static uint8_t ack_msg[12];     // used by router to acknowledge it's presence
    static uint8_t poll_msg[12];    // used by anchor to poll for a response
    static uint8_t resp_msg[15];    // used by router to respond to an anchor
    static uint8_t final_msg[24];   // used by anchor to send the final message

    void spin_router_for_one_anchor(uint8_t anchor_id);
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
    * @brief Callback to process RX timeout events
    * @param  cb_data  callback data
    * @return  none
    */
    static void rx_timeout_cb(const dwt_cb_data_t *cb_data);
    /*
    * @fn rx_error_cb()
    * @brief Callback to process RX error events
    * @param  cb_data  callback data
    * @return  none
    */
    static void rx_error_cb(const dwt_cb_data_t *cb_data);
    /*
    * @fn tx_complete_cb()
    * @brief Callback to process TX confirmation events
    * @param  cb_data  callback data
    * @return  none
    */
    static void tx_complete_cb(const dwt_cb_data_t *cb_data);
};

extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
