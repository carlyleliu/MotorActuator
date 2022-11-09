#ifndef __MIDDLEWARE_MOTOR_CONTROL_WHEELED_CHASSIS_ABSTRACT_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_WHEELED_CHASSIS_ABSTRACT_HPP__

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <cmath>

class WheeledChassisAbstract
{
  public:
    WheeledChassisAbstract(float wheel_radius = 0 , float chassis_radius = 0 ,unsigned char num = 0)
    {
        wheel_radius_ = wheel_radius;
        chassis_radius_ = chassis_radius;
        motor_num_ = num;
    };

    /**
     * @brief set wheel radius
     * @param wheel_radius set value
     * @return None
     */
    void SetWheelRadius(float wheel_radius) { wheel_radius_ = wheel_radius; };

    /**
     * @brief set chassis radius
     * @param chassis_radius set value
     * @return None
     */
    void SetChassisRadius(float chassis_radius) { chassis_radius_ = chassis_radius; };

    /**
     * @brief set global coordinat z
     * @param global_coordinat_z set value
     * @return None
     */
    void SetGlobalCoordinatZ(float global_coordinat_z) { global_coordinat_z_ = global_coordinat_z; };

    /**
     * @brief get wheel radius
     * @param None
     * @return wheel radius value
     */
    float GetWheelRadius(void) const {return wheel_radius_; };

    /**
     * @brief get chassis radius
     * @param None
     * @return chassis radius value
     */
    float GetChassisRadius(void) const {return chassis_radius_; };

    /**
     * @brief get global coordinat radius
     * @param None
     * @return global coordinat value
     */
    float GetGlobalCoordinatZ(void) const { return global_coordinat_z_; };

    /**
     * @brief set global speed for chassis
     * @param global_speed_target expect speed in global coordinat
     * @return None
     */
    void SetGlobalSpeed(Eigen::Vector3f& global_speed_target)
    {
        GlobalToMotorSpeed(global_speed_target);
    }

    /**
     * @brief set robot speed for chassis
     * @param robot_speed_target expect speed in robot coordinat
     * @return None
     */
    void SetRobotSpeed(Eigen::Vector3f& robot_speed_target)
    {
        RobotToMotorSpeed(robot_speed_target);
    }

    /**
     * @brief get robot speed for chassis
     * @param None
     * @return measure speed in robot coordinat
     */
    Eigen::Vector3f GetRobotSpeed(void)
    {
        return MotorToRobotSpeed();
    }

    /**
     * @brief get global speed for chassis
     * @param None
     * @return measure speed in global coordinat
     */
    Eigen::Vector3f GetGlobalSpeed(void)
    {
        return MotorToGlobalSpeed();
    }

    /**
     * @brief get global coordinat for chassis
     * @param None
     * @return measure position in global coordinat
     */
    Eigen::Vector3f GetGlobalCoordinate(void)
    {
        Eigen::Vector3f global_speed;

        global_speed = MotorToGlobalSpeed();
        global_coordinate_measure_ = global_speed * dt_;

        return global_coordinate_measure_;
    }

    /**
     * @brief get robot coordinat for chassis
     * @param None
     * @return measure position in robot coordinat
     */
    Eigen::Vector3f GetRobotCoordinate(void)
    {
        Eigen::Vector3f robot_speed;

        robot_speed = MotorToGlobalSpeed();
        robot_coordinate_measure_ = robot_speed * dt_;

        return robot_coordinate_measure_;
    }

  private:
    /**
     * @brief 世界坐标到机器人坐标的速度变换
     * @param global_speed 为Global坐标的速度向量xyw
     * @return Eigen::Vector3f Robot坐标的速度向量xyw
     */
    Eigen::Vector3f GlobalToRobotSpeed(Eigen::Vector3f& global_speed)
    {
        Eigen::Vector3f robot_speed;
        Eigen::Matrix3f rotate_mat;

        rotate_mat <<  cos(global_coordinat_z_), sin(global_coordinat_z_), 0.0f,
                      -sin(global_coordinat_z_), cos(global_coordinat_z_), 0.0f,
                      0.0f,                      0.0f,                     1.0f;

        robot_speed = rotate_mat * global_speed;

        return robot_speed;
    }

    /**
     * @brief 机器人坐标到世界坐标的速度变换
     * @param robot_speed Robot坐标的速度向量xyw
     * @return Eigen::Vector3f Global坐标的速度向量xyw
     */
    Eigen::Vector3f RobotToGlobalSpeed(Eigen::Vector3f& robot_speed)
    {
        Eigen::Vector3f global_speed;
        Eigen::Matrix3f rotate_mat;

        rotate_mat << cos(global_coordinat_z_), -sin(global_coordinat_z_), 0.0f,
                      sin(global_coordinat_z_),  cos(global_coordinat_z_), 0.0f,
                      0.0f,                      0.0f,                     1.0f;

        global_speed = rotate_mat * robot_speed;

        return global_speed;
    }

    /**
     * @brief 世界坐标到轮子的变换
     *
     * @param global_speed Global坐标的速度向量xyw
     * @return None
     */
    void GlobalToMotorSpeed(Eigen::Vector3f& global_speed)
    {
        RobotToMotorSpeed(GlobalToRobotSpeed(global_speed));
    }

    /**
     * @brief 轮子线速度或者线里程微分到世界坐标的变换
     *
     * @return Eigen::Vector3f Global坐标的速度向量xyw
     */
    Eigen::Vector3f MotorToGlobalSpeed(void)
    {
        return RobotToGlobalSpeed(MotorToRobotSpeed());
    }

  private:
      /**
       * @brief 机器人坐标速度到轮子线速度的变换
       *
       * @param robot_speed 机器人坐标系下的速度
       */
      virtual void RobotToMotorSpeed(Eigen::Vector3f& robot_speed) = 0;

      /**
       * @brief 轮子线速度到机器人坐标速度的变换
       *
       * @return Eigen::Vector3f 机器人坐标系下的速度
       */
      virtual Eigen::Vector3f MotorToRobotSpeed(void) = 0;

  protected:
    uint8_t motor_num_;
    float wheel_radius_;
    float body_radius_;
    float global_coordinat_z_; // 为机器人当前方向  即坐标值的Z

    uint64_t current_time_stamp_;
    uint64_t last_time_stamp_;
    double dt_;

    Eigen::Vector3f global_coordinate_measure_; // (x, y, radian)
    Eigen::Vector3f robot_coordinate_measure_; // (x, y, radian)
    Eigen::Vector3f robot_speed_measure_; // (x, y, z)
    Eigen::Vector3f global_speed_measure_; // (x, y, z)

    std::variant<Eigen::Vector2f, Eigen::Vector3f, Eigen::Vector4f> motor_angle_speed_target_;
    std::variant<Eigen::Vector2f, Eigen::Vector3f, Eigen::Vector4f> motor_angle_speed_measure_;
    std::variant<Eigen::Vector2f, Eigen::Vector3f, Eigen::Vector4f> motor_line_speed_target_;
    std::variant<Eigen::Vector2f, Eigen::Vector3f, Eigen::Vector4f> motor_line_speed_measure_;
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_WHEELED_CHASSIS_ABSTRACT_HPP__
