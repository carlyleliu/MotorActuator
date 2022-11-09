/**
 ******************************************************************************
 * @file    revup_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Rev-Up Control component of the motor Control SDK:
 *
 *          * Main Rev-Up procedure to execute programmed phases
 *          * On the Fly (OTF)
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMicROELECTRONicS AND CONTRibUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTicULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMicROELECTRONicS OR CONTRibUTORS BE LiaBLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECiaL, EXEMPLARY, OR CONSEQUENTiaL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVicES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LiaBILITY, WHETHER IN CONTRACT, STRicT LiaBILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSibILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "revup_ctrl.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup RevUpCtrl Rev-Up Control
 * @brief Rev-Up Control component of the motor Control SDK
 *
 * Used to start up the motor with a set of pre-programmed phases.
 *
 * The number of phases of the Rev up procdure can range from 0 to 5.
 * The Rev-Up controller must be called at speed loop frequency.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Timeout used to reset integral term of PLL.
 *  It is expressed in ms.
 *
 */
#define RUC_OTF_PLL_RESET_TIMEOUT 100u

/**
 * @brief  Initialize and configure the RevUpCtrl Component
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  ptr_stc: Pointer on speed and torque controller structure.
 * @param  ptr_vss: Pointer on virtual speed sensor structure.
 * @param  ptr_snsl: Pointer on sensorles state observer structure.
 * @param  ptr_pwm: Pointer on the PWM structure.
 *  @retval none
 */
__weak void ruc_init(rev_up_ctrl_t* phandle, speedn_torq_ctrl_t* ptr_stc, virtual_speed_sensor_t* ptr_vss, sto_t* ptr_snsl, pwmc_t* ptr_pwm)
{
    revup_ctrl_phase_params_t* ptr_ruc_phase_params = &phandle->params_data[0];
    uint8_t phase = 0u;

    phandle->ptr_stc = ptr_stc;
    phandle->ptr_vss = ptr_vss;
    phandle->ptr_snsl = ptr_snsl;
    phandle->ptr_pwm = ptr_pwm;
    phandle->otf_sclowside = false;
    phandle->entered_zone1 = false;

    while ((ptr_ruc_phase_params != MC_NULL) && (phase < RUC_MAX_PHASE_NUMBER)) {
        ptr_ruc_phase_params = ptr_ruc_phase_params->ptr_next;
        phase++;
    }
    phandle->params_data[phase - 1u].ptr_next = MC_NULL;

    phandle->phase_nbr = phase;

    phandle->reset_pll_th = (uint8_t)((RUC_OTF_PLL_RESET_TIMEOUT * phandle->ruc_frequency_hz) / 1000u);
}

/**
 * @brief  Initialize internal RevUp controller state.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  motor_direction: rotor rotation direction.
 *         This parameter must be -1 or +1.
 *  @retval none
 */
__weak void ruc_clear(rev_up_ctrl_t* phandle, int16_t motor_direction)
{
    virtual_speed_sensor_t* ptr_vss = phandle->ptr_vss;
    speedn_torq_ctrl_t* ptr_stc = phandle->ptr_stc;
    revup_ctrl_phase_params_t* ptr_phase_params = phandle->params_data;

    phandle->direction = motor_direction;
    phandle->entered_zone1 = false;

    /*Initializes the rev up stages counter.*/
    phandle->stage_cnt = 0u;
    phandle->otf_rel_counter = 0u;
    phandle->otf_sclowside = false;

    /* Calls the clear method of VSS.*/
    vss_clear(ptr_vss);

    /* Sets the STC in torque mode.*/
    stc_set_control_mode(ptr_stc, STC_TORQUE_MODE);

    /* Sets the mechanical starting angle of VSS.*/
    vss_set_mec_angle(ptr_vss, phandle->starting_mec_angle * motor_direction);

    /* Sets to zero the starting torque of STC */
    stc_exec_ramp(ptr_stc, 0, 0u);

    /* Gives the first command to STC and VSS.*/
    stc_exec_ramp(ptr_stc, ptr_phase_params->final_torque * motor_direction, (uint32_t)(ptr_phase_params->durationms));

    vss_set_mec_acceleration(ptr_vss, ptr_phase_params->final_mec_speed_unit * motor_direction, ptr_phase_params->durationms);

    /* Compute phase_remaining_ticks.*/
    phandle->phase_remaining_ticks =
    (uint16_t)(((uint32_t)ptr_phase_params->durationms * (uint32_t)phandle->ruc_frequency_hz) / 1000u);

    phandle->phase_remaining_ticks++;

    /*Set the next phases parameter pointer.*/
    phandle->ptr_current_phase_params = ptr_phase_params->ptr_next;

    /*Timeout counter for PLL reset during OTF.*/
    phandle->reset_pll_cnt = 0u;
}

/**
 * @brief  Main revup controller procedure executing overall programmed phases and
 *         on-the-fly startup handling.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to false when entire revup phases have been completed.
 */
__weak bool ruc_otf_exec(rev_up_ctrl_t* phandle)
{
    bool is_speed_reliable;
    bool retVal = true;
    bool condition = false;

    if (phandle->phase_remaining_ticks > 0u) {
        /* Decrease the phase_remaining_ticks.*/
        phandle->phase_remaining_ticks--;

        /* OTF start-up */
        if (phandle->stage_cnt == 0u) {
            if (phandle->entered_zone1 == false) {
                if (phandle->ptr_snsl->fct_sto_otf_reset_pll != MC_NULL) {
                    phandle->reset_pll_cnt++;
                    if (phandle->reset_pll_cnt > phandle->reset_pll_th) {
                        phandle->ptr_snsl->fct_sto_otf_reset_pll(phandle->ptr_snsl);
                        phandle->otf_rel_counter = 0u;
                        phandle->reset_pll_cnt = 0u;
                    }
                }

                is_speed_reliable = phandle->ptr_snsl->fct_sto_speed_reliability_check(phandle->ptr_snsl);

                if (is_speed_reliable) {
                    if (phandle->otf_rel_counter < 127u) {
                        phandle->otf_rel_counter++;
                    }
                } else {
                    phandle->otf_rel_counter = 0u;
                }

                if (phandle->ptr_snsl->fct_sto_otf_reset_pll != MC_NULL) {
                    if (phandle->otf_rel_counter == (phandle->reset_pll_th >> 1)) {
                        condition = true;
                    }
                } else {
                    if (phandle->otf_rel_counter == 127) {
                        condition = true;
                    }
                }

                if (condition == true) {
                    bool bCollinearSpeed = false;
                    int16_t hObsspeed_unit = spd_get_avrg_mecspeed_unit(phandle->ptr_snsl->_Super);
                    int16_t hObsspeed_unitAbsValue =
                    (hObsspeed_unit < 0 ? (-hObsspeed_unit) : (hObsspeed_unit)); /* hObsspeed_unit absolute value */

                    if (phandle->direction > 0) {
                        if (hObsspeed_unit > 0) {
                            bCollinearSpeed = true; /* actual and reference speed are collinear*/
                        }
                    } else {
                        if (hObsspeed_unit < 0) {
                            bCollinearSpeed = true; /* actual and reference speed are collinear*/
                        }
                    }

                    if (bCollinearSpeed == false) {
                        /*reverse speed management*/
                        phandle->otf_rel_counter = 0u;
                    } else /*speeds are collinear*/
                    {
                        if ((uint16_t)(hObsspeed_unitAbsValue) > phandle->min_startup_valid_speed) {
                            /* startup end, go to run */
                            phandle->ptr_snsl->fct_force_convergency1(phandle->ptr_snsl);
                            phandle->entered_zone1 = true;
                        } else if ((uint16_t)(hObsspeed_unitAbsValue) > phandle->min_startup_fly_speed) {
                            /* synch with startup*/
                            /* nearest phase search*/
                            int16_t hOldFinalMecspeed_unit = 0;
                            int16_t hOldFinalTorque = 0;
                            int32_t wDeltaSpeedRevUp;
                            int32_t wDeltaTorqueRevUp;
                            bool bError = false;
                            vss_set_copy_observer(phandle->ptr_vss);
                            phandle->ptr_snsl->fct_force_convergency2(phandle->ptr_snsl);

                            if (phandle->ptr_current_phase_params == MC_NULL) {
                                bError = true;
                                phandle->phase_remaining_ticks = 0u;
                            } else {
                                while (phandle->ptr_current_phase_params->final_mec_speed_unit < hObsspeed_unitAbsValue) {
                                    if (phandle->ptr_current_phase_params->ptr_next == MC_NULL) {
                                        /* sets for Revup fail error*/
                                        bError = true;
                                        phandle->phase_remaining_ticks = 0u;
                                        break;
                                    } else {
                                        /* skips this phase*/
                                        hOldFinalMecspeed_unit = phandle->ptr_current_phase_params->final_mec_speed_unit;
                                        hOldFinalTorque = phandle->ptr_current_phase_params->final_torque;
                                        phandle->ptr_current_phase_params = phandle->ptr_current_phase_params->ptr_next;
                                        phandle->stage_cnt++;
                                    }
                                }
                            }

                            if (bError == false) {
                                /* calculation of the transition phase from OTF to standard revup */
                                int16_t hTorqueReference;

                                wDeltaSpeedRevUp = (int32_t)(phandle->ptr_current_phase_params->final_mec_speed_unit)
                                                   - (int32_t)(hOldFinalMecspeed_unit);
                                wDeltaTorqueRevUp =
                                (int32_t)(phandle->ptr_current_phase_params->final_torque) - (int32_t)(hOldFinalTorque);

                                hTorqueReference =
                                (int16_t)((((int32_t)hObsspeed_unit) * wDeltaTorqueRevUp) / wDeltaSpeedRevUp) + hOldFinalTorque;

                                stc_exec_ramp(phandle->ptr_stc, hTorqueReference, 0u);

                                phandle->phase_remaining_ticks = 1u;

                                phandle->ptr_current_phase_params = &phandle->otf_phase_params;

                                phandle->stage_cnt = 6u;
                            } /* no MC_NULL error */
                        }     /* speed > MinStartupFly */
                        else {
                        }
                    } /* speeds are collinear */
                }     /* speed is reliable */
            }         /*entered_zone1 1 is false */
            else {
                phandle->ptr_snsl->fct_force_convergency1(phandle->ptr_snsl);
            }
        } /*stage 0*/
    }     /* phase_remaining_ticks > 0 */

    if (phandle->phase_remaining_ticks == 0u) {
        if (phandle->ptr_current_phase_params != MC_NULL) {
            if (phandle->stage_cnt == 0u) {
                /*end of OTF*/
                pwmc_switch_off_pwm(phandle->ptr_pwm);
                phandle->otf_sclowside = true;
                PWMC_TurnOnLowSides(phandle->ptr_pwm);
                phandle->otf_rel_counter = 0u;
            } else if ((phandle->stage_cnt == 1u)) {
                pwmc_switch_on_pwm(phandle->ptr_pwm);
                phandle->otf_sclowside = false;
            } else {
            }

            /* If it becomes zero the current phase has been completed.*/
            /* Gives the next command to STC and VSS.*/
            stc_exec_ramp(phandle->ptr_stc, phandle->ptr_current_phase_params->final_torque * phandle->direction,
                          (uint32_t)(phandle->ptr_current_phase_params->durationms));

            vss_set_mec_acceleration(phandle->ptr_vss, phandle->ptr_current_phase_params->final_mec_speed_unit * phandle->direction,
                                     phandle->ptr_current_phase_params->durationms);

            /* Compute phase_remaining_ticks.*/
            phandle->phase_remaining_ticks =
            (uint16_t)(((uint32_t)phandle->ptr_current_phase_params->durationms * (uint32_t)phandle->ruc_frequency_hz) / 1000u);
            phandle->phase_remaining_ticks++;

            /*Set the next phases parameter pointer.*/
            phandle->ptr_current_phase_params = phandle->ptr_current_phase_params->ptr_next;

            /*Increases the rev up stages counter.*/
            phandle->stage_cnt++;
        } else {
            if (phandle->stage_cnt == phandle->phase_nbr - 1) /* End of user programmed revup */
            {
                retVal = false;
            } else if (phandle->stage_cnt == 7u) /* End of first OTF runs */
            {
                phandle->stage_cnt = 0u; /* Breaking state */
                phandle->phase_remaining_ticks = 0u;
            } else {
            }
        }
    }
    return retVal;
}

/**
 * @brief  Main revup controller procedure executing overall programmed phases.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to false when entire revup phases have been completed.
 */
__weak bool ruc_exec(rev_up_ctrl_t* phandle)
{
    bool ret_val = true;

    if (phandle->phase_remaining_ticks > 0u) {
        /* Decrease the phase_remaining_ticks.*/
        phandle->phase_remaining_ticks--;

    } /* phase_remaining_ticks > 0 */

    if (phandle->phase_remaining_ticks == 0u) {
        if (phandle->ptr_current_phase_params != MC_NULL) {
            /* If it becomes zero the current phase has been completed.*/
            /* Gives the next command to STC and VSS.*/
            stc_exec_ramp(phandle->ptr_stc, phandle->ptr_current_phase_params->final_torque * phandle->direction,
                          (uint32_t)(phandle->ptr_current_phase_params->durationms));

            vss_set_mec_acceleration(phandle->ptr_vss, phandle->ptr_current_phase_params->final_mec_speed_unit * phandle->direction,
                                     phandle->ptr_current_phase_params->durationms);

            /* Compute phase_remaining_ticks.*/
            phandle->phase_remaining_ticks =
            (uint16_t)(((uint32_t)phandle->ptr_current_phase_params->durationms * (uint32_t)phandle->ruc_frequency_hz) / 1000u);
            phandle->phase_remaining_ticks++;

            /*Set the next phases parameter pointer.*/
            phandle->ptr_current_phase_params = phandle->ptr_current_phase_params->ptr_next;

            /*Increases the rev up stages counter.*/
            phandle->stage_cnt++;
        } else {
            ret_val = false;
        }
    }
    return ret_val;
}

/**
 * @brief  Provide current state of revup controller procedure.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to true when entire revup phases have been completed.
 */
__weak bool ruc_completed(rev_up_ctrl_t* phandle)
{
    bool ret_val = false;
    if (phandle->ptr_current_phase_params == MC_NULL) {
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief  Allow to exit from RevUp process at the current rotor speed.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval none
 */
__weak void ruc_stop(rev_up_ctrl_t* phandle)
{
    virtual_speed_sensor_t* ptr_vss = phandle->ptr_vss;
    phandle->ptr_current_phase_params = MC_NULL;
    phandle->phase_remaining_ticks = 0u;
    vss_set_mec_acceleration(ptr_vss, spd_get_avrg_mecspeed_unit(&ptr_vss->_Super), 0u);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif

/**
 * @brief  Check that alignement and first acceleration stage are completed.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to true when first acceleration stage has been reached.
 */
__weak bool ruc_first_acceleration_stage_reached(rev_up_ctrl_t* phandle)
{
    bool ret_val = false;

    if (phandle->stage_cnt >= phandle->first_acceleration_stage) {
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief  Allow to modify duration of a selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  phase: RevUp phase where new duration shall be modified.
 *         This parameter must be a number between 0 and 6.
 * @param  durationms: new duration value required for associated phase.
 *         This parameter must be set in millisecond.
 *  @retval none
 */
__weak void ruc_set_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase, uint16_t durationms)
{
    phandle->params_data[bPhase].durationms = durationms;
}

/**
 * @brief  Allow to modify targetted mechanical speed of a selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  bPhase: RevUp phase where new mechanical speed shall be modified.
 *         This parameter must be a number between 0 and 6.
 * @param  final_mec_speed_unit: new targetted mechanical speed.
 *         This parameter must be expressed in 0.1Hz.
 *  @retval none
 */
__weak void ruc_set_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_mec_speed_unit)
{
    phandle->params_data[bPhase].final_mec_speed_unit = final_mec_speed_unit;
}

/**
 * @brief  Allow to modify targetted the motor torque of a selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  bPhase: RevUp phase where new the motor torque shall be modified.
 *         This parameter must be a number between 0 and 6.
 * @param  final_torque: new targetted motor torque.
 *  @retval none
 */
__weak void ruc_set_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_torque)
{
    phandle->params_data[bPhase].final_torque = final_torque;
}

/**
 * @brief  Allow to read duration set in selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  bPhase: RevUp phase from where duration is read.
 *         This parameter must be a number between 0 and 6.
 *  @retval Returns duration used in selected phase.
 */
__weak uint16_t ruc_get_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase)
{
    return ((uint16_t)phandle->params_data[bPhase].durationms);
}

/**
 * @brief  Allow to read targetted rotor speed set in selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  bPhase: RevUp phase from where targetted rotor speed is read.
 *         This parameter must be a number between 0 and 6.
 *  @retval Returns targetted rotor speed set in selected phase.
 */
__weak int16_t ruc_get_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase)
{
    return ((int16_t)phandle->params_data[bPhase].final_mec_speed_unit);
}

/**
 * @brief  Allow to read targetted motor torque set in selected phase.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 * @param  bPhase: RevUp phase from where targetted motor torque is read.
 *         This parameter must be a number between 0 and 6.
 *  @retval Returns targetted motor torque set in selected phase.
 */
__weak int16_t ruc_get_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase)
{
    return ((int16_t)phandle->params_data[bPhase].final_torque);
}

/**
 * @brief  Allow to read total number of programmed phases.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Returns number of phases relative to the programmed revup.
 */
__weak uint8_t ruc_get_number_of_phases(rev_up_ctrl_t* phandle)
{
    return ((uint8_t)phandle->phase_nbr);
}

/**
 * @brief  Allow to read status of On The Fly (OTF) feature.
 * @param  phandle: Pointer on Handle structure of RevUp controller.
 *  @retval Boolean set to true at the end of OTF precessing.
 */
__weak bool ruc_get_sclowside_otf_status(rev_up_ctrl_t* phandle)
{
    return (phandle->otf_sclowside);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
