#include <pid_controller.hpp>

/**
 * @brief  It Reset the PID controller
 * @param  None
 * @retval None
 */
void PidController::ResetController(void)
{
    integral_term_ = 0.0f;
    prev_process_var_error_ = 0.0f;
}

/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  process_var_error: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
FloatPoint PidController::PIController(FloatPoint process_var_error)
{
    /* Integral term computation */
    integral_term_ += process_var_error;

    return kp_gain_ * process_var_error + ki_gain_ * integral_term_;
}

/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  phandle: handler of the current instance of the PID component
 * @param  process_var_error: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */
FloatPoint PidController::PIDController(FloatPoint process_var_error)
{
    FloatPoint differential_term, delta_error;

    if (kd_gain_ != 0) /* derivative terms not used */
    {
        delta_error = process_var_error - prev_process_var_error_;
        differential_term = kd_gain_ * delta_error;

        prev_process_var_error_ = process_var_error;

        return PIController(process_var_error) + differential_term;
    } else {
        return PIController(process_var_error);
    }
}

