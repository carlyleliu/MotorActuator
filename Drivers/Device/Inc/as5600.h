#ifndef DRIVE_DEVICE_AS5600_H_
#define DRIVE_DEVICE_AS5600_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "peripherals.h"

enum AS5600Registers {
    /* set i2c address */
    ams5600_address = 0x36,
    zmco = 0x00,
    zpos_hi = 0x01,
    zpos_lo = 0x02,
    mpos_hi = 0x03,
    mpos_lo = 0x04,
    mang_hi = 0x05,
    mang_lo = 0x06,
    conf_hi = 0x07,
    conf_lo = 0x08,
    raw_ang_hi = 0x0c,
    raw_ang_lo = 0x0d,
    ang_hi = 0x0e,
    ang_lo = 0x0f,
    stat = 0x0b,
    agc = 0x1a,
    mag_hi = 0x1b,
    mag_lo = 0x1c,
    burn = 0xff
};

uint16_t AS5600GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_AS5600_H_ */
