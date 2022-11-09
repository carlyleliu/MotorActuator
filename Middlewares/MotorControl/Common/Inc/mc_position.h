#ifndef _MC_POSITION_H_
#define _MC_POSITION_H_

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "abs_encoder.h"
#include "mc_type.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define motor_MAX_SPEED_RPM 1000

/* Do ACC duration calculate rate: (Speed loop time * SPEED_ACC_RATE) */
#define SPEED_ACC_RATE 20

#define _IQ24(A) (long)((A)*16777216.0L)
#define IQ24_PI  52707178 // 3.14159265358979323846 * 16777216

/* PI_MUL = 2*PI*10000 */
#define PI_MUL 62832

/* Define low delta angle and high delta angle, used for control mode change */
#define CHANGE_LIMIT_LOW  15700 / 2
#define CHANGE_LIMIT_HIGH 15700 // 62832*2 1570--90

/* Define positon control structure */
typedef struct position {
    abs_encoder_t* ptr_abs_encoder; /* abs encoder sensor pointer */

    volatile int32_t current_rpm; /* current speed with rpm */

    volatile int32_t target_rpm; /* target rpm rpm */

    volatile int32_t current_rpm_filter; /* after filter current speed with rpm rpm */

    volatile int32_t target_pos; /* target position in position loop */

    volatile int32_t current_pos; /* current position in position loop */

    volatile int32_t current_pos_imu; /* current position in position loop with imu */

    volatile int32_t error_pos; /* error position in position loop */

    uint32_t pos_ticks; /* current ticks in position loop */

    uint32_t last_pos_ticks; /* last ticks in position loop */

    bool mode_flag; /* mode_flag = 1, under torque mode, mode_flag = 0, under speed mode */

    bool torque_first_flag; /* First torque mode flag */

} position_t;

/* Define mode */
#define P_TORQUE_MODE 1
#define P_SPEED_MODE  0

#define MAX_POSITION PI_MUL
#define MIN_POSITION 0

#define MAX_SPEED_RPM (240)
#define MIN_SPEED_RPM (-240)

void position_init(position_t* pHandl, abs_encoder_t* abs_enc);
int32_t position_get_error_angle(position_t* phandle);
int16_t position_calc_speed_referrence(position_t* phandle);

void position_set_target_pos(position_t* phandle, int32_t angle_ref);
void position_set_target_speed(position_t* phandle, int32_t speed_ref);
void position_get_error_angle_by_imu(position_t* phandle);
void position_set_imu_angle(position_t* phandle, int32_t current_angle);

#define POSITION_CONTROL

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
