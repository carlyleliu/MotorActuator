#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Sample, 7);
constexpr uint32_t ktestTimes = 2000;

int main(void)
{
	printk("start sample\n");

    float phase = 0.0f;
    float rpm = 0.0f;
    float sensor_update_time = 0.0f;
    uint32_t times = 0;

    BldcMotor* bldc_ptr = new(BldcMotor);

    ImplPwm* pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    ImplPwm* pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    ImplPwm* pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    FieldOrientedController& foc_ptr = bldc_ptr->GetFocHandle();
    OpenLoopController* open_loop_control_ptr = new(OpenLoopController);

    AbsoluteAngleEncoder* encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pwm0_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_w_);

    open_loop_control_ptr->SetAlignMode(true);
    open_loop_control_ptr->SetMaxCurrentRamp(2);
    open_loop_control_ptr->SetMaxVoltageRamp(5);
    open_loop_control_ptr->SetMaxPhaseVelRamp(5);
    open_loop_control_ptr->SetVelocityTarget(0);

    open_loop_control_ptr->SetCurrentTarget(0);
    open_loop_control_ptr->SetVoltageTarget(5);
    open_loop_control_ptr->SetInitialPhase(0);

    while (1) {
        k_busy_wait(300);
        open_loop_control_ptr->Update();
        phase = open_loop_control_ptr->GetTargetPhase();
        rpm = open_loop_control_ptr->GetTargetPhaseVelocity();
        sensor_update_time = time();

        foc_ptr.SetPhaseAngle(phase);
        foc_ptr.SetPhaseVelocity(rpm);
        foc_ptr.SetSensorUpdateTime(sensor_update_time);
        foc_ptr.SetVbus(12);
        foc_ptr.SetVdq(open_loop_control_ptr->GetVdqTarget());

        bldc_ptr->MotorRun();
        pwm0_ptr->Update();
        pwm1_ptr->Update();
        pwm2_ptr->Update();
        times++;
        if (times > ktestTimes)
        {
            LOG_INF("angle = %d\n", encoder_ptr->Align());
        }
	}

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();

    delete(bldc_ptr);
    delete(open_loop_control_ptr);
    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    return 1;
}
