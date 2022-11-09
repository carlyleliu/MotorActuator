#ifndef __ALGORITHM_FOC_CONTROLLER_HPP__
#define __ALGORITHM_FOC_CONTROLLER_HPP__

#include <component_port.hpp>
#include <pid_controller.hpp>

#include <time_util.h>

#include <optional>
#include <string>
#include <vector>
#include <array>
#include <tuple>

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
    void FocClark(std::optional<float2D> current);
    void FocPark(float theta);
    void FocRevPark(std::optional<float2D> v_dq, float theta);
    std::tuple<float, float, float, bool> FocSVM(void);

    std::optional<float2D> GetIqdMeasure(void) {
        return i_dq_measured_;
    };

  private:
    std::optional<float2D> i_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_target_;
    std::optional<float2D> i_dq_measured_;
};

#endif // ! __ALGORITHM_FOC_CONTROLLER_HPP__

