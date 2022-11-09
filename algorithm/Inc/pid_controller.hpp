#ifndef __ALGORITHM_PID_CONTROLLER_HPP__
#define __ALGORITHM_PID_CONTROLLER_HPP__

#include <driver.h>
#include <syslog.h>

using FloatPoint = float;

/**
 * @brief Handle of a PID component
 *
 * @detail This structure stores all the parameters needed to perform a proportional,
 * integral and derivative regulation computation. It also stores configurable limits
 * in order to saturate the integral terms and the output value. This structure is
 * passed to each PID component function.
 */
class PidController
{
  public:
    PidController():
        kp_gain_(0.0f),
        ki_gain_(0.0f),
        kd_gain_(0.0f),
        integral_term_(0.0f),
        prev_process_var_error_(0.0f) {};
    ~PidController() {};

  public:
    /**
     * It updates the Kp gain
     */
    void SetKp(FloatPoint kp) {
        kp_gain_ = kp;
    };

    /**
     * It updates the Ki gain
     */
    void SetKi(FloatPoint ki) {
        ki_gain_ = ki;
    };

    /**
     * It updates the Kd gain
     */
    void SetKd(FloatPoint kd) {
        kd_gain_ = kd;
    };

    /**
     * It updates the Kp ki and kd gain
     */
    void SetParameter(FloatPoint kp, FloatPoint ki, FloatPoint kd) {kp_gain_ = kp; ki_gain_ = ki; kd_gain_ = kd; };

    /**
     * This function reset pid controller param
     * proportional and integral_terms
     */
    void ResetController(void);

    /**
     * This function compute the output of a PI regulator sum of its
     * proportional and integral_terms
     */
    FloatPoint PIController(FloatPoint process_var_error);

    /**
     * This function compute the output of a PID regulator sum of its
     *  proportional, integral and derivative terms
     */
    FloatPoint PIDController(FloatPoint process_var_error);

  private:
    FloatPoint kp_gain_;                 /*< gain used by PID component */
    FloatPoint ki_gain_;                 /*< gain used by PID component */
    FloatPoint kd_gain_;                 /*< Kd gain used by PID component */
    FloatPoint integral_term_;           /*< integral term */
    FloatPoint prev_process_var_error_;  /*< previous process variable used by the derivative part of the PID component */
};

#endif // ! __ALGORITHM_PID_CONTROLLER_HPP__
