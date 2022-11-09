#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>
#include <motor_control_factory.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Main, 7);
constexpr uint32_t ktestTimes = 2000;

int main(void)
{
	printk("start main\n");

    float sensor_update_time = 0.0f;

    PM3505 pm3505;
    BldcMotor& pm3505_bldc = pm3505.Create();

    ImplPwm* pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    ImplPwm* pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    ImplPwm* pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    FieldOrientedController& foc_ptr = pm3505_bldc.GetFocHandle();

    AbsoluteAngleEncoder* encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pwm0_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_w_);
    pm3505_bldc.phase_measure_.ConnectTo(&encoder_ptr->measure_normalize_angle_);
    pm3505_bldc.phase_velocity_measure_.ConnectTo(&encoder_ptr->measure_velocity_);

    pm3505_bldc.SetControlType(MOTOR_CONTROL_TYPE_TORQUE);
    pm3505_bldc.SetTorque(0.1);

    while (1) {
        k_busy_wait(300);
        sensor_update_time = time();
        encoder_ptr->Update();

        foc_ptr.SetSensorUpdateTime(sensor_update_time);
        foc_ptr.SetVbus(12);

        pm3505_bldc.MotorTask();
        pwm0_ptr->Update();
        pwm1_ptr->Update();
        pwm2_ptr->Update();

	}

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();
    pm3505_bldc.phase_measure_.DisConnect();
    pm3505_bldc.phase_velocity_measure_.DisConnect();

    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    return 1;
}
