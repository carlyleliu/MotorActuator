#ifndef __MIDDLEWARD_MOTOR_CONTROL_CHASSIS_DIFFERENTAL_WHEEL_CHASSIS_HPP__
#define __MIDDLEWARD_MOTOR_CONTROL_CHASSIS_DIFFERENTAL_WHEEL_CHASSIS_HPP__

#include <wheeled_chassis_abstract.hpp>

/*******************************************************************************
2wd (Top view)

1   X    2
*******************************************************************************/

/*******************************************************************************
4wd (Top view)

1       2
  X   X
    X
  X   X
3       4
*******************************************************************************/

class DifferentialWheelChassis : public WheeledChassisAbstract
{
  public:
    DifferentialWheelChassis(float wheel_radius = 0  , float body_radius = 0 , uint8_t num = 2):
        WheeledChassisAbstract(wheel_radius, body_radius , num){}

  private:
    /**
     * @brief 机器人坐标速度到轮子线速度的变换
     *        Vl = -Vx + (Vz * D) / 2
     *        Vr = Vx + (Vz * D) / 2
     * @param robot_speed 机器人坐标系下的速度
     * @return None
     */
    void RobotToMotorSpeed(Eigen::Vector3f& robot_speed)
    {
        motor_line_speed_target_(0) =  -1 * robot_speed(0)  + 0 * robot_speed(1) + body_radius_ * robot_speed(2);
        motor_line_speed_target_(1) =   1 * robot_speed(0)  + 0 * robot_speed(1) + body_radius_ * robot_speed(2);

        if(motor_num_ >= 4) {
            motor_line_speed_target_(2) = motor_line_speed_target_(0);
            motor_line_speed_target_(3) = motor_line_speed_target_(1);
        }
    }

    /**
     * @brief 轮子线速度到机器人坐标速度的变换
     *        Vx = (Vl + Vr) / 2
     *        Vz = (Vr - Vl) / D
     * @return Eigen::Vector3f 机器人坐标系下的速度
     */
    Eigen::Vector3f MotorToRobotSpeed(void)
    {
        Eigen::Vector3f robot_speed;

        robot_speed(0) = -0.5f * motor_line_speed_measure_(0) + 0.5f * motor_line_speed_measure_(1);
        robot_speed(1) = 0 * motor_line_speed_measure_(0) + 0 * motor_line_speed_measure_(1);
        robot_speed(2) = (0.5f / body_radius_) * (motor_line_speed_measure_(0) + motor_line_speed_measure_(1));

        return robot_speed;
    }
};

#endif // ! __MIDDLEWARD_MOTOR_CONTROL_CHASSIS_DIFFERENTAL_WHEEL_CHASSIS_HPP__
