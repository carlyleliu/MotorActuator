#ifndef __MIDDLEWARE_MOTOR_CONTROL_CHASSIS_MECANUM4_WHEEL_CHASSIS_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_CHASSIS_MECANUM4_WHEEL_CHASSIS_HPP__

#include <wheeled_chassis_abstract.hpp>

// support for 4 wheel mecanum robots
// body_radius = a + b

/*******************************************************************************
4wd (Top view)

1       2
  X   X
    X
  X   X
4       3
*******************************************************************************/

class Mecanum4WheelChassis : public WheeledChassisAbstract
{
public:
    Mecanum4WheelChassis(float wheel_radius = 0  , float body_radius = 0) :
        WheeledChassisAbstract(wheel_radius , body_radius  , 4){}

private:
    void RobotToMotorSpeed(Eigen::Vector3f& robot_speed)
    {
        Eigen::MatrixXf rotare_mat(4,3);
        rotare_mat << -1,   1,  body_radius,
                       1,   1,  body_radius,
                       1,   -1, body_radius,
                      -1,   -1, body_radius;

        motor_line_speed_measure_ = rotare_mat *  robot_speed;
    }

    Eigen::Vector3f MotorToRobotSpeed(void)
    {
        Eigen::Vector3f robot_speed;
        Eigen::MatrixXf rotare_mat(3,4);

        rotare_mat << -0.25,            0.25,               0.25,               -0.25,
                      0.25,             0.25,               -0.25,              -0.25,
                      0.25/body_radius, 0.25/body_radius,   0.25/body_radius,   0.25/body_radius;

        robot_speed = rotare_mat * motor_line_speed_measure_;

        return robot_speed;
    }
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_CHASSIS_MECANUM4_WHEEL_CHASSIS_HPP__
