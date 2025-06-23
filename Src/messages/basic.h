/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#include <cstdint>

typedef struct {
    uint8_t* data;
    uint8_t len;
} Message;

struct BasicMessage {
    uint8_t msg_type;
    uint8_t sn;
    uint8_t payload[0];
};

#define BASIC_MSG_TYPE 0x01

#define BASIC_MSG_SN_IDX 0

#define BASIC_MSG_LEN_MAX 127

#define BASIC_MSG_PAYLOAD_LEN(msg) ((msg)->payload[BASIC_MSG_SN_IDX])

#define BASIC_MSG_PAYLOAD(msg) (&(msg)->payload[BASIC_MSG_SN_IDX + 1])

inline void pack_basic_msg(Message* msg, BasicMessage* basic_msg) {

}