#ifndef __DEVICE_ICM42688_HPP__
#define __DEVICE_ICM42688_HPP__

#include <imu_abstract.hpp>
class Icm42688 : public ImuAbstract
{
  public:
    Icm42688();
    ~Icm42688();

    int Init(void);
    int DeInit(void);
    int Read(void);

  private:
    int Process(void);
#ifdef CONFIG_ICM42688_TRIGGER
    int Trigger(void);
#endif /* CONFIG_ICM42688_TRIGGER */

  private:
#ifdef CONFIG_ICM42688_TRIGGER

#endif /* CONFIG_ICM42688_TRIGGER */
};

#endif // ! __DEVICE_ICM42688_HPP__
