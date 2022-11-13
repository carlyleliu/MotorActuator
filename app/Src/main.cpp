

/* Includes ------------------------------------------------------------------*/
#include <driver.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>
#include <motor_control_factory.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

#include <driver.h>

PM3505 pm3505;
ImplPwm* pwm0_ptr = nullptr;
ImplPwm* pwm1_ptr = nullptr;
ImplPwm* pwm2_ptr = nullptr;
AbsoluteAngleEncoder* encoder_ptr = nullptr;

int MotorControlInit(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pwm0_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_w_);
    pm3505_bldc.normalize_angle_measure_.ConnectTo(&encoder_ptr->normalize_angle_measure_);
    pm3505_bldc.velocity_measure_.ConnectTo(&encoder_ptr->velocity_measure_);
    pm3505_bldc.sensor_update_time_.ConnectTo(&encoder_ptr->sensor_update_time_);

    pm3505_bldc.SetControlType(MOTOR_CONTROL_TYPE_SPEES);
    //pm3505_bldc.SetTorque(0.5);
    pm3505_bldc.SetVelocity(-60);

    LOG_INF("init finished\n");

    return 0;
}

int MotorControlDeInit(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();
    pm3505_bldc.normalize_angle_measure_.DisConnect();
    pm3505_bldc.velocity_measure_.DisConnect();

    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    LOG_INF("deinit finished\n");

    return 0;
}

void MotorControlUpdate(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    encoder_ptr->Update();

    pm3505_bldc.MotorTask();
    pwm0_ptr->Update();
    pwm1_ptr->Update();
    pwm2_ptr->Update();
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    PeripheralsInit();

    MotorControlInit();

    SetItCb(MotorControlUpdate);

    while (1) {
        //MotorControlUpdate();
        HAL_Delay(1000);
    }

    MotorControlDeInit();
}