#ifndef __ALGORITHM_FOC_CONTROLLER_HPP__
#define __ALGORITHM_FOC_CONTROLLER_HPP__

#include <pid_controller.hpp>

#include <string>
#include <array>

#include <driver.h>
#include <syslog.h>

#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u

#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

using float2D = std::pair<float, float>;

class FieldOrientedController
{
  public:
    void FocClark(std::array<float, 2>& current);
    void FocPark(float theta);
    void FocRevPark(std::array<float, 2>& v_dq, float theta);
    //std::tuple<float, float, float, bool> FocSVM(void);
    bool FocSVM(float* tA, float* tB, float* tC);

    std::array<float, 2>& GetIqdMeasure(void) {
        return i_dq_measured_;
    };

  private:
    std::array<float, 2> i_alpha_beta_measured_;
    std::array<float, 2> v_alpha_beta_measured_;
    std::array<float, 2> v_alpha_beta_target_;
    std::array<float, 2> i_dq_measured_;
};

#endif // ! __ALGORITHM_FOC_CONTROLLER_HPP__

