
/**
 ******************************************************************************
 * @file    regular_conversion_manager.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the regular_conversion_manager component of the motor Control SDK:
 *           Register conversion with or without callback
 *           Execute regular conv directly from Temperature and VBus sensors
 *           Execute user regular conversion scheduled by medium frequency task
 *           Manage user conversion state machine
 *           +
 *           +
 *
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
 */

/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup RCM Regular Conversion Manager
 * @brief Regular Conversion Manager component of the motor Control SDK
 *
 * motorControl SDK makes an extensive usage of ADCs. Some conversions are timing critical
 * like current reading, and some have less constraints. If an ADC offers both Injected and Regular,
 * channels, critical conversions will be systematically done on Injected channels, because they
 * interrupt any ongoing regular conversion so as to be executed without delay.
 * Others conversions, mainly Bus voltage, and Temperature sensing are performed with regular channels.
 * If users wants to perform ADC conversions with an ADC already used by MC SDK, they must use regular
 * conversions. It is forbidden to use Injected channel on an ADC that is already in use for current reading.
 * As usera and MC-SDK may share ADC regular scheduler, this component intents to manage all the
 * regular conversions.
 *
 * If users wants to execute their own conversion, they first have to register it through the
 * rcm_register_reg_conv_with_cb() or rcm_register_reg_conv() APIs. Multiple conversions can be registered,
 * but only one can be scheduled at a time .
 *
 * A requested user regular conversion will be executed by the medium frequency task after the
 * MC-SDK regular safety conversions: Bus voltage and Temperature.
 *
 * If a callback is registered, particular care must be taken with the code executed inside the CB.
 * The callback code is executed under Medium frequency task IRQ context (Systick).
 *
 * If the Users do not register a callback, they must poll the RCM state machine to know if
 * a conversion is ready to be read, scheduled, or free to be scheduled. This is performed through
 * the rcm_get_user_conv_state() API.
 *
 * If the state is RCM_USERCONV_IDLE, a conversion is ready to be scheduled.
 * if a conversion is already scheduled, the returned value is RCM_USERCONV_REQUESTED.
 * if a conversion is ready to be read, the returned value is RCM_USERCONV_EOC.
 * In RCM_USERCONV_EOC state, a call to rcm_get_user_conv will consume the value, and set the state machine back
 * to RCM_USERCONV_IDLE state. It implies that a second call without new conversion performed,
 * will send back 0xffff which is an error value meaning that the data is not available.
 * If a conversion request is executed, but the previous conversion has not been completed, nor consumed,
 * the request is discarded and the rcm_request_user_conv return false.
 *
 * If a callback is registered, the data read is sent back to the callback parameters, and therefor consumed.
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief Document as stated in template.h
 *
 * ...
 */
typedef enum { notvalid, ongoing, valid } rcm_status_t;

typedef struct {
    bool enable;
    rcm_status_t status;
    uint16_t value;
    uint8_t prev;
    uint8_t next;
} rcm_no_inj_t;

typedef struct {
    rcm_exec_cb_t cb;
    void* data;
} rcm_callback_t;

// pwmc_r3_2_t* pwmc;

/* Private defines -----------------------------------------------------------*/
/**
 * @brief Number of regular conversion allowed By default.
 *
 * In single drive configuration, it is defined to 4. 2 of them are consumed by
 * Bus voltage and temperature reading. This leaves 2 handles available for
 * user conversions
 *
 * In dual drives configuration, it is defined to 6. 2 of them are consumed by
 * Bus voltage and temperature reading for each motor. This leaves 2 handles
 * available for user conversion.
 *
 * Defined to 4 here.
 */
#define RCM_MAX_CONV 4

/* Global variables ----------------------------------------------------------*/

reg_conv_t* rcm_handle_array[RCM_MAX_CONV];
rcm_callback_t rcm_cb_array[RCM_MAX_CONV];

rcm_no_inj_t rcm_no_inj_array[RCM_MAX_CONV];
uint8_t rcm_current_handle;
uint8_t rcm_userconv_handle;
uint16_t rcm_user_conv_value;
rcm_user_conv_state_t rcm_user_conv_state;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Registers a regular conversion, and attaches a callback.
 *
 * This function registers a regular ADC conversion that can be later scheduled for execution. It
 * returns a handle that uniquely identifies the conversion. This handle is used in the other API
 * of the Regular Converion Manager to reference the registered conversion.
 *
 * A regular conversion is defined by an ADC + ADC channel pair. If a registration already exists
 * for the requested ADC + ADC channel pair, the same handle will be reused.
 *
 * The regular conversion is registered along with a callback that is executed each time the
 * conversion has completed. The callback is invoked with two parameters:
 *
 * - the handle of the regular conversion
 * - a data pointer, supplied by uthe users at registration time.
 *
 * The registration may fail if there is no space left for additional conversions. The
 * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
 *
 * @note   Users who do not want a callback to be executed at the end of the conversion,
 *         should use rcm_register_reg_conv() instead.
 *
 * @param  regConv Pointer to the regular conversion parameters.
 *         Contains ADC, Channel and sampling time to be used.
 *
 * @param  fctCB Function called once the regular conversion is executed.
 *
 * @param  data Used to save a user context. this parameter will be send back by
 *               the fctCB function. @b Note: This parameter can be NULL if not used.
 *
 *  @retval the handle of the registered conversion or 255 if the registration failed
 */
uint8_t rcm_register_reg_conv_with_cb(reg_conv_t* regConv, rcm_exec_cb_t fctCB, void* data)
{
    uint8_t handle;
    handle = rcm_register_reg_conv(regConv);
    if (handle < RCM_MAX_CONV) {
        rcm_cb_array[handle].cb = fctCB;
        rcm_cb_array[handle].data = data;
    }
    return handle;
}

/**
 * @brief  Registers a regular conversion.
 *
 * This function registers a regular ADC conversion that can be later scheduled for execution. It
 * returns a handle that uniquely identifies the conversion. This handle is used in the other API
 * of the Regular Converion Manager to reference the registered conversion.
 *
 * A regular conversion is defined by an ADC + ADC channel pair. If a registration already exists
 * for the requested ADC + ADC channel pair, the same handle will be reused.
 *
 * The registration may fail if there is no space left for additional conversions. The
 * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
 *
 * @note   Users who do not want a callback to be executed at the end of the conversion,
 *         should use rcm_register_reg_conv() instead.
 *
 * @param  regConv Pointer to the regular conversion parameters.
 *         Contains ADC, Channel and sampling time to be used.
 *
 *  @retval the handle of the registered conversion or 255 if the registration failed
 */
uint8_t rcm_register_reg_conv(reg_conv_t* regConv)
{
    uint8_t handle = 255;
    uint8_t i = 0;

    /** Parse the array to be sure that same
     * conversion does not already exist*/
    while (i < RCM_MAX_CONV) {
        if (rcm_handle_array[i] == 0 && handle > RCM_MAX_CONV) {
            handle = i; /* First location available, but still loptr_ping to check that this config does not already exist*/
        }
        /* Ticket 64042 : If rcm_handle_array [i] is null access to data member will cause Memory Fault. */
        if (rcm_handle_array[i] != 0) {
            if ((rcm_handle_array[i]->channel == regConv->channel) && (rcm_handle_array[i]->reg_adc == regConv->reg_adc)) {
                handle = i;       /* Reuse the same handle */
                i = RCM_MAX_CONV; /* we can skip the rest of the loop*/
            }
        }
        i++;
    }
    if (handle < RCM_MAX_CONV) {
        rcm_handle_array[handle] = regConv;
        rcm_cb_array[handle].cb = NULL; /* if a previous callback was attached, it is cleared*/
        if (LL_ADC_IsEnabled(regConv->reg_adc) == 0) {
            LL_ADC_DisableIT_EOC(regConv->reg_adc);
            LL_ADC_ClearFlag_EOC(regConv->reg_adc);
            LL_ADC_DisableIT_JEOC(regConv->reg_adc);
            LL_ADC_ClearFlag_JEOC(regConv->reg_adc);

            LL_ADC_StartCalibration(regConv->reg_adc, LL_ADC_SINGLE_ENDED);
            while (LL_ADC_IsCalibrationOnGoing(regConv->reg_adc)) {
            }
            /* ADC Enable (must be done after calibration) */
            /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0
             * following a calibration phase, could have no effect on ADC
             * within certain AHB/ADC clock ratio.
             */
            while (LL_ADC_IsActiveFlag_ADRDY(regConv->reg_adc) == 0) {
                LL_ADC_Enable(regConv->reg_adc);
            }

        } else {
        }
        /* conversion handler is created, will be enabled by the first call to rcm_exec_regular_conv*/
        rcm_no_inj_array[handle].enable = false;
        rcm_no_inj_array[handle].next = handle;
        rcm_no_inj_array[handle].prev = handle;
        /* reset regular conversion sequencer length set by cubeMX */
        LL_ADC_REG_SetSequencerLength(regConv->reg_adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
        /* configure the sampling time (should already be configured by for non user conversions)*/
        LL_ADC_SetChannelSamplingTime(regConv->reg_adc, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel), regConv->sampling_time);
    } else {
        /* Nothing to do handle is already set to error value : 255 */
    }
    return handle;
}

/**
 * This function is used to read the result of a regular conversion.
 * Depending of the MC state machine, this function can poll on the ADC end of conversion or not.
 * If the ADC is already in use for currents sensing, the regular conversion can not
 * be executed instantaneously but have to be scheduled in order to be executed after currents sensing
 * inside HF task.
 * This function takes care of inserting the handle into the scheduler.
 * If it is possible to execute the conversion instantaneously, it will be executed, and result returned.
 * Otherwise, the latest stored conversion result will be returned.
 *
 * NOTE: This function is not part of the public API and users should not call it.
 */
uint16_t rcm_exec_regular_conv(uint8_t handle)
{
    uint16_t ret_val;
    uint8_t former_next;
    uint8_t i = 0;
    uint8_t last_enable = RCM_MAX_CONV;

    if (rcm_no_inj_array[handle].enable == false) {
        /* find position in the list */
        while (i < RCM_MAX_CONV) {
            if (rcm_no_inj_array[i].enable == true) {
                if (rcm_no_inj_array[i].next > handle)
                /* We found a previous reg conv to link with */
                {
                    former_next = rcm_no_inj_array[i].next;
                    rcm_no_inj_array[handle].next = former_next;
                    rcm_no_inj_array[handle].prev = i;
                    rcm_no_inj_array[i].next = handle;
                    rcm_no_inj_array[former_next].prev = handle;
                    i = RCM_MAX_CONV; /* stop the loop, handler inserted*/
                } else {              /* We found an enabled regular conv,
                                       * but do not know yet if it is the one we have to be linked to. */
                    last_enable = i;
                }
            } else { /* nothing to do */
            }
            i++;
            if (i == RCM_MAX_CONV)
            /* We reach end of the array without handler inserted*/
            {
                if (last_enable != RCM_MAX_CONV)
                /* we find a regular conversion with smaller position to be linked with */
                {
                    former_next = rcm_no_inj_array[last_enable].next;
                    rcm_no_inj_array[handle].next = former_next;
                    rcm_no_inj_array[handle].prev = last_enable;
                    rcm_no_inj_array[last_enable].next = handle;
                    rcm_no_inj_array[former_next].prev = handle;
                } else { /* the current handle is the only one in the list */
                    /* previous and next are already pointing to itself (done at registerRegConv) */
                    rcm_current_handle = handle;
                }
            } else {
                /* Nothing to do we are parsing the array, nothing inserted yet*/
            }
        }
        /* The handle is now linked with others, we can set the enable flag */
        rcm_no_inj_array[handle].enable = true;
        rcm_no_inj_array[handle].status = notvalid;
        if (rcm_no_inj_array[rcm_current_handle].status
            != ongoing) { /* select the new conversion to be the next scheduled only if a conversion is not ongoing*/
            rcm_current_handle = handle;
        }
    } else {
        /* Nothing to do the current handle is already scheduled */
    }
    // if (pwmc->adc_regular_locked == false) //TODO
    /* The ADC is free to be used asynchronously*/
    {
        LL_ADC_REG_SetSequencerRanks(rcm_handle_array[handle]->reg_adc, LL_ADC_REG_RANK_1,
                                     __LL_ADC_DECIMAL_NB_TO_CHANNEL(rcm_handle_array[handle]->channel));

        LL_ADC_REG_ReadConversionData12(rcm_handle_array[handle]->reg_adc);
        /* Start ADC conversion */
        LL_ADC_REG_StartConversion(rcm_handle_array[handle]->reg_adc);

        /* Wait EOC */
        while (LL_ADC_IsActiveFlag_EOC(rcm_handle_array[handle]->reg_adc) == RESET) {
        }

        /* Read the "Regular" conversion (Not related to current sampling) */
        rcm_no_inj_array[handle].value = LL_ADC_REG_ReadConversionData12(rcm_handle_array[handle]->reg_adc);
        rcm_current_handle = rcm_no_inj_array[handle].next;
        rcm_no_inj_array[handle].status = valid;
    }
    ret_val = rcm_no_inj_array[handle].value;
    return ret_val;
}

/**
 * @brief Schedules a regular conversion for execution.
 *
 * This function requests the execution of the user-defined regular conversion identified
 * by @p handle. All user defined conversion requests must be performed inside routines with the
 * same priority level. If a previous regular conversion request is pending this function has no
 * effect, for this reason is better to call rcm_get_user_conv_state() and check if the state is
 * #RCM_USERCONV_IDLE before calling rcm_request_user_conv().
 *
 * @param  handle used for the user conversion.
 *
 * @return true if the regular conversion could be scheduled and false otherwise.
 */
bool rcm_request_user_conv(uint8_t handle)
{
    bool ret_val = false;
    if (rcm_user_conv_state == RCM_USERCONV_IDLE) {
        rcm_userconv_handle = handle;
        /* must be done last so that rcm_userconv_handle already has the right value */
        rcm_user_conv_state = RCM_USERCONV_REQUESTED;
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief  Returns the last user-defined regular conversion that was executed.
 *
 * This function returns a valid result if the state returned by
 * rcm_get_user_conv_state is #RCM_USERCONV_EOC.
 *
 * @retval uint16_t The converted value or 0xFFFF in case of conversion error.
 */
uint16_t rcm_get_user_conv(void)
{
    uint16_t ret_val = 0xFFFFu;
    if (rcm_user_conv_state == RCM_USERCONV_EOC) {
        ret_val = rcm_user_conv_value;
        rcm_user_conv_state = RCM_USERCONV_IDLE;
    }
    return ret_val;
}

/**
 *  This function must be scheduled by mc_task.
 *  It executes the current user conversion that has been selected by the
 *  latest call to rcm_request_user_conv
 *
 * NOTE: This function is not part of the public API and users should not call it.
 */
void rcm_exec_user_conv()
{
    if (rcm_user_conv_state == RCM_USERCONV_REQUESTED) {
        rcm_user_conv_value = rcm_exec_regular_conv(rcm_userconv_handle);
        rcm_user_conv_state = RCM_USERCONV_EOC;
        if (rcm_cb_array[rcm_userconv_handle].cb != NULL) {
            rcm_user_conv_state = RCM_USERCONV_IDLE;
            rcm_cb_array[rcm_userconv_handle].cb(rcm_userconv_handle, rcm_user_conv_value,
                                                 rcm_cb_array[rcm_userconv_handle].data);
        }
    }
}

/**
 * @brief  Returns the status of the last requested regular conversion.
 *
 * It can be one of the following values:

 * - UDRC_STATE_IDLE no regular conversion request pending.
 * - UDRC_STATE_REQUESTED regular conversion has been requested and not completed.
 * - UDRC_STATE_EOC regular conversion has been completed but not readed from the user.
 *
 * @retval The state of the last user-defined regular conversion.
 */
rcm_user_conv_state_t rcm_get_user_conv_state(void)
{
    return rcm_user_conv_state;
}

/**
 * @brief  Un-schedules a regular conversion
 *
 * This function does not poll ADC read and is meant to be used when
 * ADCs do not support injected channels.
 *
 * In such configurations, once a regular conversion has been executed once,
 * It is continuously scheduled in HF task after current reading.
 *
 * This function remove the handle from the scheduling.
 *
 * @note Note that even though, in such configurations, regular conversions are
 *       continuously scheduled after having been requested once, the results of
 *       subsequent conversions are not made available unless the users invoke
 *       rcm_request_user_conv() again.
 *
 */
bool rcm_pause_regular_conv(uint8_t handle)
{
    bool ret_val;
    uint8_t prev;
    uint8_t next;

    if (handle < RCM_MAX_CONV) {
        ret_val = true;
        if (rcm_no_inj_array[handle].enable == true) {
            rcm_no_inj_array[handle].enable = false;
            rcm_no_inj_array[handle].status = notvalid;
            prev = rcm_no_inj_array[handle].prev;
            next = rcm_no_inj_array[handle].next;
            rcm_no_inj_array[prev].next = rcm_no_inj_array[handle].next;
            rcm_no_inj_array[next].prev = rcm_no_inj_array[handle].prev;
        } else {
        }
    } else {
        ret_val = false;
    }
    return ret_val;
}

/**
 * Starts the ext scheduled regular conversion
 *
 * This function does not poll on ADC read and is foreseen to be used inside
 * high frequency task where ADC are shared between currents reading
 * and user conversion.
 *
 * NOTE: This function is not part of the public API and users should not call it.
 */
void rcm_exec_next_conv(void)
{
    if (rcm_no_inj_array[rcm_current_handle].enable == true) {
        /* When this function is called, the ADC conversions triggered by External
       event for current reading has been completed.
       ADC is therefore ready to be started because already stopped.*/

        /* Clear EOC */
        LL_ADC_ClearFlag_EOC(ADC1);
        LL_ADC_REG_SetSequencerRanks(rcm_handle_array[rcm_current_handle]->reg_adc, LL_ADC_REG_RANK_1,
                                     __LL_ADC_DECIMAL_NB_TO_CHANNEL(rcm_handle_array[rcm_current_handle]->channel));

        LL_ADC_REG_ReadConversionData12(rcm_handle_array[rcm_current_handle]->reg_adc);

        /* Start ADC for regular conversion */
        LL_ADC_REG_StartConversion(rcm_handle_array[rcm_current_handle]->reg_adc);
        rcm_no_inj_array[rcm_current_handle].status = ongoing;
    } else {
        /* nothing to do, conversion not enabled have already notvalid status */
    }
}

/**
 * Reads the result of the ongoing regular conversion
 *
 * This function is foreseen to be used inside
 * high frequency task where ADC are shared between current reading
 * and user conversion.
 *
 * NOTE: This function is not part of the public API and users should not call it.
 */
void rcm_read_ongoing_conv(void)
{
    if (rcm_no_inj_array[rcm_current_handle].status == ongoing) {
        /* Reading of ADC Converted Value */
        rcm_no_inj_array[rcm_current_handle].value =
        LL_ADC_REG_ReadConversionData12(rcm_handle_array[rcm_current_handle]->reg_adc);
        rcm_no_inj_array[rcm_current_handle].status = valid;
        /* Restore back DMA configuration. */
    }

    /* Prepare next conversion */
    rcm_current_handle = rcm_no_inj_array[rcm_current_handle].next;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
