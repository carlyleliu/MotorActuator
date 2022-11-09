/* Includes ------------------------------------------------------------------*/
#include "abs_encoder.h"

#include "stdlib.h"

#define NEGATIVE (int8_t)(-1)
#define POSITIVE (int8_t)1

/**
 * @brief  Clear software FIFO where are "pushed" latest speed information
 *         This function must be called before starting the motor to initialize
 *         the speed measurement process.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component*
 * @retval none
 */
static void abs_encoder_clean(abs_encoder_t* phandle)
{
    /* Reset speed reliability */
    phandle->sensor_reliable = true;

    /* acceleration measurement not implemented.*/
    phandle->_Super.mec_accel_unit_p = 0;

    phandle->direction = POSITIVE;

    /* Clear speed error counter */
    phandle->_Super.speed_error_number = 0;
}

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVic)
            required for the speed position sensor management using HALL
            sensors.
  * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
void abs_encoder_init(abs_encoder_t* phandle)
{
    phandle->sensor_reliable = true;
    phandle->mec_offset_to_el_dpp = (int16_t)(0);
    phandle->mec_to_dpp = 1;

    if (NULL != phandle->abs_encoder_device_init) {
        phandle->abs_encoder_device_init();
    }
    abs_encoder_clean(phandle);
}

/**
 * @brief  Update the rotor electrical angle integrating the last measured
 *         instantaneous electrical speed express in dpp.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval int16_t Measured electrical angle in s16degree format.
 */

inline int16_t abs_encoder_calc_el_angle(abs_encoder_t* phandle)
{
    if (phandle->sensor_reliable) {
        phandle->last_mec_angle_dpp = phandle->mec_angle_dpp;
        phandle->mec_angle = phandle->abs_encoder_read_angle();
        phandle->mec_angle_dpp = (phandle->mec_angle * phandle->mec_to_dpp);
        phandle->el_angle_dpp = (phandle->mec_angle_dpp - phandle->mec_offset_to_el_dpp) * phandle->_Super.el_to_mec_ratio;

        if (phandle->mec_angle_dpp - phandle->last_mec_angle_dpp < -32768) {
            phandle->circle_counter++;
        } else if (phandle->mec_angle_dpp - phandle->last_mec_angle_dpp > 32768) {
            phandle->circle_counter--;
        }
    }

    phandle->_Super.el_angle = phandle->el_angle_dpp;
    phandle->_Super.mec_angle = phandle->mec_angle_dpp;
    return phandle->_Super.el_angle;
}

/**
 * @brief  Update the rotor electrical angle integrating the last measured
 *         instantaneous electrical speed express in dpp.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval int16_t Measured electrical angle in s16degree format.
 */
inline bool abs_encoder_reliable(abs_encoder_t* phandle)
{
    return phandle->sensor_reliable;
}
