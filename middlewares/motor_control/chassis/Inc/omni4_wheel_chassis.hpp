#ifndef __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI4_WHEEL_CHASSIS_HPP__
#define __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI4_WHEEL_CHASSIS_HPP__

#include <wheeled_chassis_abstract.hpp>

//support for 4 wheel omni robots

/*******************************************************************************
4wd (Top view)

1       2
  X   X
    X
  X   X
4       3
*******************************************************************************/

class Omni4WheelChassis : public WheeledChassisAbstract
{
public:
    Omni4WheelChassis(float wheel_radius  = 0 , float body_radius = 0) :
        WheeledChassisAbstract(wheel_radius , body_radius  , 4){}

private:
    void RobotToMotorSpeed(Eigen::Vector3f& robot_speed)
    {
        Eigen::MatrixXf rotare_mat(4,3);

        rotare_mat << 0,    1,  body_radius,
                      -1,   0,  body_radius,
                      0,    -1, body_radius,
                      1,    0,  body_radius;

        motor_line_speed_measure_ = rotare_mat *  robot_speed;
    }

    Eigen::Vector3f MotorToRobotSpeed(void)
    {
        Eigen::Vector3f robot_speed;
        Eigen::MatrixXf rotare_mat(3,4);

        rotare_mat << 0,                    -0.5,               0,                  0.5,
                      0.5,                  -0,                 -0.5,               0,
                      0.25f/body_radius,    0.25f/body_radius,  0.25f/body_radius,  0.25f/body_radius;

        robot_speed = rotare_mat * motor_line_speed_measure_;

        return robot_speed;
    }

};

#endif // ! __MIDDLEWARE_MOTORCONTROL_CHASSIS_OMNI4_WHEEL_CHASSIS_HPP__
