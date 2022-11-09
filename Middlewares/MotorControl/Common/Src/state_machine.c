/**
 ******************************************************************************
 * @file    state_machine.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the motor Control State Machine component of the motor Control SDK:
 *
 *           * Check that transition from one state to another is legal
 *           * Handle the fault processing
 *           * Provide accessor to State machine internal state
 *           * Provide accessor to error state
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
#include "state_machine.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup STATE_MACHINE motor Control State Machine
 * @brief motor Control State Machine component of the motor Control SDK
 *
 * @todo Document the motor Control State Machine "module".
 *
 * @{
 */

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param phandle pointer on the component instance to initialize.
 * @retval none.
 */
__weak void stm_init(state_machine_t* phandle)
{
    phandle->state = IDLE;
    phandle->fault_now = MC_NO_FAULTS;
    phandle->fault_occurred = MC_NO_FAULTS;
}

/**
 * @brief It submits the request for moving the state machine into the state
 *        specified by state (FAULT_NOW and FAUL_OVER are not handled by this
 *        method). Accordingly with the current state, the command is really
 *        executed (state machine set to state) or discarded (no state
 *        changes).
 *        If requested state can't be reached the return value is false and the
 *        MC_SW_ERROR is raised, but if requested state is IDLE_START,
 *        IDLE_ALIGNMENT or ANY_STOP, that corresponds with the user actions:
 *        Start motor, Encoder Alignemnt and Stop motor, the MC_SW_ERROR is
 *        not raised.
 * @param pHanlde pointer of type  state_machine_t.
 * @param state New requested state
 * @retval bool It returns true if the state has been really set equal to
 *         state, false if the requested state can't be reached
 */
__weak bool stm_next_state(state_machine_t* phandle, State_t state)
{
    bool change_state = false;
    State_t current_state = phandle->state;
    State_t new_state = current_state;

    switch (current_state) {
        case ICLWAIT:
            if (state == IDLE) {
                new_state = state;
                change_state = true;
            }
            break;
        case IDLE:
            if ((state == IDLE_START) || (state == IDLE_ALIGNMENT) || (state == ICLWAIT)) {
                new_state = state;
                change_state = true;
            }
            break;

        case IDLE_ALIGNMENT:
            if ((state == ANY_STOP) || (state == ALIGN_CHARGE_BOOT_CAP) || (state == ALIGN_OFFSET_CALIB)) {
                new_state = state;
                change_state = true;
            }
            break;

        case ALIGN_CHARGE_BOOT_CAP:
            if ((state == ALIGN_OFFSET_CALIB) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case ALIGN_OFFSET_CALIB:
            if ((state == ALIGN_CLEAR) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case ALIGN_CLEAR:
            if ((state == ALIGNMENT) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case ALIGNMENT:
            if (state == ANY_STOP) {
                new_state = state;
                change_state = true;
            }
            break;

        case IDLE_START:
            if ((state == ANY_STOP) || (state == CHARGE_BOOT_CAP) || (state == START) || (state == OFFSET_CALIB)
                || (state == IDLE_ALIGNMENT)) {
                new_state = state;
                change_state = true;
            }
            break;

        case CHARGE_BOOT_CAP:
            if ((state == OFFSET_CALIB) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case OFFSET_CALIB:
            if ((state == CLEAR) || (state == ANY_STOP) || (state == WAIT_STOP_motor)) {
                new_state = state;
                change_state = true;
            }
            break;

        case WAIT_STOP_motor:
            if ((state == CLEAR) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case CLEAR:
            if ((state == START) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case START:
            if ((state == SWITCH_OVER) || (state == ANY_STOP) || (state == START_RUN)) {
                new_state = state;
                change_state = true;
            }
            break;

        case SWITCH_OVER:
            if ((state == START) || (state == ANY_STOP) || (state == START_RUN)) {
                new_state = state;
                change_state = true;
            }
            break;

        case START_RUN:
            if ((state == RUN) || (state == ANY_STOP)) {
                new_state = state;
                change_state = true;
            }
            break;

        case RUN:
            if (state == ANY_STOP) {
                new_state = state;
                change_state = true;
            }
            break;

        case ANY_STOP:
            if (state == STOP) {
                new_state = state;
                change_state = true;
            }
            break;

        case STOP:
            if (state == STOP_IDLE) {
                new_state = state;
                change_state = true;
            }
            break;

        case STOP_IDLE:
            if ((state == IDLE) || (state == ICLWAIT)) {
                new_state = state;
                change_state = true;
            }
            break;
        default:
            break;
    }

    if (change_state) {
        phandle->state = new_state;
    } else {
        if (!((state == IDLE_START) || (state == IDLE_ALIGNMENT) || (state == ANY_STOP))) {
            /* If new state is not a user command START/STOP raise a software error */
            stm_fault_processing(phandle, MC_SW_ERROR, 0u);
        }
    }

    return (change_state);
}

/**
 * @brief It clocks both hw and SW faults processing and update the state
 *        machine accordingly with set_errors, reset_errors and present state.
 *        Refer to State_t description for more information about fault states.
 * @param pHanlde pointer of type  state_machine_t
 * @param set_errors Bit field reporting faults currently present
 * @param reset_errors Bit field reporting faults to be cleared
 * @retval State_t New state machine state after fault processing
 */
__weak State_t stm_fault_processing(state_machine_t* phandle, uint16_t set_errors, uint16_t reset_errors)
{
    State_t local_state = phandle->state;

    /* Set current errors */
    phandle->fault_now = (phandle->fault_now | set_errors) & (~reset_errors);
    phandle->fault_occurred |= set_errors;

    if (local_state == FAULT_NOW) {
        if (phandle->fault_now == MC_NO_FAULTS) {
            phandle->state = FAULT_OVER;
            local_state = FAULT_OVER;
        }
    } else {
        if (phandle->fault_now != MC_NO_FAULTS) {
            phandle->state = FAULT_NOW;
            local_state = FAULT_NOW;
        }
    }

    return (local_state);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Returns the current state machine state
 * @param  pHanlde pointer of type  state_machine_t
 * @retval State_t Current state machine state
 */
__weak State_t stm_get_state(state_machine_t* phandle)
{
    return (phandle->state);
}

/**
 * @brief It reports to the state machine that the fault state has been
 *        acknowledged by the user. If the state machine is in FAULT_OVER state
 *        then it is moved into STOP_IDLE and the bit field variable containing
 *        information about the faults historically occured is cleared.
 *        The method call is discarded if the state machine is not in FAULT_OVER
 * @param pHanlde pointer of type  state_machine_t
 * @retval bool true if the state machine has been moved to IDLE, false if the
 *        method call had no effects
 */
__weak bool stm_fault_acknowledged(state_machine_t* phandle)
{
    bool tobe_returned = false;

    if (phandle->state == FAULT_OVER) {
        phandle->state = STOP_IDLE;
        phandle->fault_occurred = MC_NO_FAULTS;
        tobe_returned = true;
    }

    return (tobe_returned);
}

/**
 * @brief It returns two 16 bit fields containing information about both faults
 *        currently present and faults historically occurred since the state
 *        machine has been moved into state
 * @param pHanlde pointer of type  state_machine_t.
 * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
 *         the information about currently present faults. In the least
 *         significant half are stored the information about the faults
 *         historically occurred since the state machine has been moved into
 *         FAULT_NOW state
 */
__weak uint32_t stm_get_fault_state(state_machine_t* phandle)
{
    uint32_t local_fault_state;

    local_fault_state = (uint32_t)(phandle->fault_occurred);
    local_fault_state |= (uint32_t)(phandle->fault_now) << 16;

    return local_fault_state;
}

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
