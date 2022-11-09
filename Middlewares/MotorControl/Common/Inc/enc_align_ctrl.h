/**
 ******************************************************************************
 * @file    enc_align_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Encoder Alignment Control component of the motor Control SDK.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 * @ingroup EncAlignCtrl
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCALIGNCTRLCLASS_H
#define __ENCALIGNCTRLCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "encoder_speed_pos_fdbk.h"
#include "mc_type.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup EncAlignCtrl
 * @{
 */

/**
 * @brief  This structure is used to handle an instance of  EncAlignCtrl component
 */

typedef struct {
    speedn_torq_ctrl_t* ptr_stc;     /*!< Speed and torque controller object used by  EAC.*/
    virtual_speed_sensor_t* ptr_vss; /*!< Virtual speed sensor object used by EAC.*/
    encoder_t* ptr_enc;              /*!< Encoder object used by EAC.*/
    uint16_t remaining_ticks;        /*!< Number of clock events remaining to complete
                                      the alignment.*/
    bool enc_aligned;                /*!< This flag is true if the encoder has been
                                      aligned at least one time, false if hasn't been
                                      never aligned.*/
    bool enc_restart;                /*!< This flag is used to force a restart of the
                                      motorafter the encoder alignment. It is true
                                      if a restart is programmed else false*/
    uint16_t eac_frequency_hz;       /*!< Frequency expressed in Hz at which the user
                                      clocks the EAC calling eac_exec method */
    int16_t final_torque;            /*!< motor torque reference imposed by STC at the
                                      end of programmed alignment. This value
                                      represents actually the Iq current expressed in
                                      digit.*/
    int16_t el_angle;                /*!< Electrical angle of programmed alignment
                                      expressed in s16degrees.*/
    uint16_t durationms;             /*!< Duration of the programmed alignment expressed
                                      in milliseconds.*/
    uint8_t el_to_mec_ratio;         /*!< Coefficient used to transform electrical to
                                      mechanical quantities and viceversa. It usually
                                      coincides with motor pole pairs number*/
} enc_align_t;

/* Exported functions ------------------------------------------------------- */

/*  Function used to initialize an instance of the EncAlignCtrl component */
void eac_init(enc_align_t* phandle, speedn_torq_ctrl_t* ptr_stc, virtual_speed_sensor_t* ptr_vss, encoder_t* ptr_enc);

/* Function used to start the encoder alignment procedure.*/
void eac_start_alignment(enc_align_t* phandle);

/* Function used to clocks the encoder alignment controller*/
bool eac_exec(enc_align_t* phandle);

/* It returns true if the encoder has been aligned at least one time*/
bool eac_aligned(enc_align_t* phandle);

/* It sets a restart after an encoder alignment*/
void eac_set_restart_state(enc_align_t* phandle, bool restart);

/* Returns true if a restart after an encoder alignment has been requested*/
bool eac_get_restart_state(enc_align_t* phandle);

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __ENCALIGNCTRLCLASS_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
