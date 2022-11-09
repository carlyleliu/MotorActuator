#include <icm42688.hpp>

#include <stdio.h>
#include <inttypes.h>

/**
 * @brief process imu data
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Process(void)
{
    time_ = time();

	return 0;
}

#ifdef CONFIG_ICM42688_TRIGGER
int Icm42688::Trigger(void)
{
	int rc = Process(dev_);

	if (rc != 0) {
		LOG_ERR("cancelling trigger due to failure: %d\n", rc);
	}

    return rc;
}
#endif /* CONFIG_ICM42688_TRIGGER */

/**
 * @brief init imu sensor
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Init(void)
{
#ifdef CONFIG_ICM42688_TRIGGER

#endif // CONFIG_ICM42688_TRIGGER
    inited_ = 1;
    LOG_INF("Icm42688Init OK!\n");

    return 0;
}

/**
 * @brief deinit imu sensor
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::DeInit(void)
{
    inited_ = 0;

    return 0;
}

/**
 * @brief read imu data
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Read(void)
{
#ifdef CONFIG_ICM42688_TRIGGER
	return Process(dev_);
#endif /* CONFIG_ICM42688_TRIGGER */
    return 0;
}
