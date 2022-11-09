#ifndef __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI3_WHEEL_CHASSIS_HPP__
#define __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI3_WHEEL_CHASSIS_HPP__

#include <wheeled_chassis_abstract.hpp>

//support for 3 wheel omni robots

/*******************************************************************************
3wd (Top view)

    1
  X   X
    X
 2     3
*******************************************************************************/

class Omni3WheelChass : public WheeledChassisAbstract
{
public:
    Omni3WheelChass(float wheel_radius = 0  , float body_radius = 0):
      WheeledChassisAbstract(wheel_radius , body_radius  , 3){}

private:
    void RobotToMotorSpeed(Eigen::Vector3f& robot_speed)
    {
        Eigen::Matrix3f rotare_mat;

        rotare_mat << 0,            1,      body_radius,
                      -0.866025,   -0.5,    body_radius,
                      0.866025,    -0.5,    body_radius;

        motor_line_speed_measure_ = rotare_mat * robot_speed;
    }

    Eigen::Vector3f MotorToRobotSpeed(void)
    {
        Eigen::Vector3f robot_speed;
        Eigen::Matrix3f rotare_mat;

        rotare_mat << 0,                        -0.57735,   0.57735,
                      0.666667,                 -0.333333,  -0.333333,
                      0.333333 / body_radius,   1,          1;

        robot_speed = rotare_mat * motor_line_speed_measure_;

        return robot_speed;
    }

};

#endif // ! __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI3_WHEEL_CHASSIS_HPP__
