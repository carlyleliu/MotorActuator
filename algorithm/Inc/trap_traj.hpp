#ifndef __ALGORITHM_TRAP_TRAJ_HPP__
#define __ALGORITHM_TRAP_TRAJ_HPP__

#include <cmath>
#include <algorithm_utils.hpp>

class TrapezoidalTrajectory
{
  public:
    struct Config_t {
        float vel_limit = 2.0f;   // [turn/s]
        float accel_limit = 0.5f; // [turn/s^2]
        float decel_limit = 0.5f; // [turn/s^2]
    };

    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    bool PlanTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    Step_t Eval(float t);

    Config_t config_;

    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tf_;

    float yAccel_;

    float t_;
};

#endif // ! __ALGORITHM_TRAP_TRAJ_HPP__
