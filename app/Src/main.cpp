

/* Includes ------------------------------------------------------------------*/
#include <driver.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>
#include <motor_control_factory.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

PM3505* pm3505_ptr = nullptr;
ImplPwm* pwm0_ptr = nullptr;
ImplPwm* pwm1_ptr = nullptr;
ImplPwm* pwm2_ptr = nullptr;
AbsoluteAngleEncoder* encoder_ptr = nullptr;

int MotorControlInit(void)
{
    BldcMotor& pm3505_bldc = pm3505_ptr->Create();

    pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pm3505_bldc.SetControlType(MOTOR_CONTROL_TYPE_SPEES);
    //pm3505_bldc.SetTorque(0.5);
    pm3505_bldc.SetVelocity(30);

    LOG_INF("init finished\n");

    return 0;
}

int MotorControlDeInit(void)
{
    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    LOG_INF("deinit finished\n");

    return 0;
}

void MotorControlUpdate(void)
{
    BldcMotor& pm3505_bldc = pm3505_ptr->Create();

    encoder_ptr->Update();

    pm3505_bldc.SetNormalizeAngleMeasure(encoder_ptr->GetNormalizeAngleMeasure());
    pm3505_bldc.SetVelocityMeasure(encoder_ptr->GetVelocityMeasure());
    pm3505_bldc.SetSensorUpdateTime(encoder_ptr->GetSensorUpdateTime());

    pm3505_bldc.MotorTask();

    pwm0_ptr->SetPwmPlus(pm3505_bldc.GetPhaseU());
    pwm1_ptr->SetPwmPlus(pm3505_bldc.GetPhaseV());
    pwm2_ptr->SetPwmPlus(pm3505_bldc.GetPhaseW());

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

    PM3505 pm3505;
    pm3505_ptr = &pm3505;

    MotorControlInit();

    SetItCb(MotorControlUpdate);

    while (1) {
        HAL_Delay(1000);
    }

    //MotorControlDeInit();
}