/**
 * imu.c — IMU660RC DMP 陀螺仪
 *
 *   SPI0: B18(SCK) B17(MOSI) B19(MISO) A2(CS) B24(INT2)
 *   上电后调用 imu_init(), 主循环每 10ms 调 imu_update()
 */
#include "zf_device_imu660rc.h"
#include "imu.h"

uint8_t imu_init(void)
{
    return imu660rc_init(IMU660RC_QUARTERNION_60HZ);   /* 60Hz DMP */
}

void imu_update(void)
{
    imu660rc_get_quarternion();   /* 更新 imu660rc_yaw/pitch/roll */
}

float imu_yaw(void)   { return imu660rc_yaw; }
float imu_pitch(void) { return imu660rc_pitch; }
float imu_roll(void)  { return imu660rc_roll; }
