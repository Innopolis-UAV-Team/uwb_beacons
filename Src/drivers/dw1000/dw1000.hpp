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

class DW1000 {
 public:
    int init();
    void spin();
};

extern DW1000 dw1000;

#endif  // SRC_DRIVERS_DW1000_DW1000_HPP_
