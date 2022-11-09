#ifndef FRAMEWORK_MOTOR_CONTROL_PARAMETER_H
#define FRAMEWORK_MOTOR_CONTROL_PARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "device.h"
#include "mc_tuning.h"
#include "mc_type.h"
#include "platform.h"
#include "pwm_curr_fdbk.h"
#include "r3_2_g4xx_pwm_curr_fdbk.h"

#define SYS_TICK_FREQUENCY 2000

/* mos charge parameter */
#define CHARGE_BOOT_CAP_MS     10
#define CHARGE_BOOT_CAP_MS2    10
#define OFFCALIBRWAIT_MS       0
#define OFFCALIBRWAIT_MS2      0
#define STOPPERMANENCY_MS      400
#define STOPPERMANENCY_MS2     400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS) / 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2) / 1000)
#define OFF_CALIBR_WAIT_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS) / 1000)
#define OFF_CALIBR_WAIT_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2) / 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS) / 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / 1000)

/* PQD conversion factor */
#define ADC_REFERENCE_VOLTAGE 3.30
#define RSHUNT                0.33000
#define AMPLIFICATION_GAIN    1.53
#define PQD_CONVERSION_FACTOR (int32_t)((1000 * 3 * ADC_REFERENCE_VOLTAGE) / (1.732 * RSHUNT * AMPLIFICATION_GAIN))

#define UD_VOLTAGE_THRESHOLD_V       5
#define ADC_REFERENCE_VOLTAGE        3.30
#define VBUS_PARTITIONING_FACTOR     0.0625
#define OV_VOLTAGE_THRESHOLD_V       36
#define ADC_TRIG_CONV_LATENCY_CYCLES 3.5
#define ADC_SAR_CYCLES               12.5

#define OVERVOLTAGE_THRESHOLD_d \
    (uint16_t)(OV_VOLTAGE_THRESHOLD_V * 65535 / (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d \
    (uint16_t)((UD_VOLTAGE_THRESHOLD_V * 65535) / ((uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR)))

#define HW_DEAD_TIME_NS 550
#define DEADTIME_NS     HW_DEAD_TIME_NS
#define DT_COMP_CNT     (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define T_ON_NS         500
#define T_OFF_NS        500
#define T_ON            (uint16_t)((T_ON_NS * ADV_TIM_CLK_MHz) / 2000)
#define T_OFF           (uint16_t)((T_OFF_NS * ADV_TIM_CLK_MHz) / 2000)

/* Motor Parameter */
#define POLE_PAIR_NUM    7 /* Number of motor pole pairs */
#define PWM_FREQ_SCALING 1

/* CircleLimit Parameter */
#define START_INDEX 63
#define MAX_MODULE  32767

#define MMITABLE                                                                                                              \
    {                                                                                                                         \
        32767, 32390, 32146, 31907, 31673, 31444, 31220, 31001, 30787, 30577, 30371, 30169, 29971, 29777, 29587, 29400,       \
        29217, 29037, 28861, 28687, 28517, 28350, 28185, 28024, 27865, 27709, 27555, 27404, 27256, 27110, 26966, 26824,       \
        26685, 26548, 26413, 26280, 26149, 26019, 25892, 25767, 25643, 25521, 25401, 25283, 25166, 25051, 24937, 24825,       \
        24715, 24606, 24498, 24392, 24287, 24183, 24081, 23980, 23880, 23782, 23684, 23588, 23493, 23400, 23307, 23215, 23125 \
    }

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif