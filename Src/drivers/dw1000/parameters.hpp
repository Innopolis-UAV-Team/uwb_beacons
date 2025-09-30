
/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Anastasiia Stepanova <asiiapine@gmail.com>
*/

#ifndef SRC_DRIVERS_DW1000_PARAMETERS_HPP_
#define SRC_DRIVERS_DW1000_PARAMETERS_HPP_

#define PARAMS_SIZE 4
enum ParametersNames {
    FLASH_NAME,
    ANTENNA_DELAY,
    REFERENCE_PGdly,
    REFERENCE_PG_COUNT,
    REFERENCE_TEMP,
    REFERENCE_POWER,
    BEST_CALIBRATION_ERROR,
    NUM_PARAMETERS
};

inline size_t get_param_offset(ParametersNames param) {
    return param * PARAMS_SIZE;
}

#endif  // SRC_DRIVERS_DW1000_PARAMETERS_HPP_
