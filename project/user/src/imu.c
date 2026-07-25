/**
 * imu.c — IMU660RC 陀螺仪 (SPI0)
 *
 *   SPI0: SCK=B18 MOSI=B17 MISO=B19 CS=A2 INT2=B24
 *   Yaw/Pitch/Roll: DMP内部融合, 直接读
 *   Gyro/Acc: 原始值+低通
 */
#include "zf_device_imu660rc.h"
#include "imu.h"

#define ALPHA  3      /* α*10 = 3, α=0.3 */

static int16_t fgx, fgy, fgz, fax, fay, faz;

static int16_t lowpass(int16_t raw, int16_t *prev) {
    int16_t f = (int16_t)(((int32_t)ALPHA * raw + (10 - ALPHA) * (*prev)) / 10);
    *prev = f;
    return f;
}

uint8_t imu_init(void) {
    return imu660rc_init(IMU660RC_QUARTERNION_60HZ);   /* 0=成功 */
}

void imu_update(void) {
    imu660rc_get_quarternion();      /* 更新 yaw/pitch/roll (DMP融合) */
    imu660rc_get_gyro();
    imu660rc_get_acc();
    lowpass(imu660rc_gyro_x, &fgx);
    lowpass(imu660rc_gyro_y, &fgy);
    lowpass(imu660rc_gyro_z, &fgz);
    lowpass(imu660rc_acc_x,  &fax);
    lowpass(imu660rc_acc_y,  &fay);
    lowpass(imu660rc_acc_z,  &faz);
}

float   imu_yaw(void)   { return imu660rc_yaw; }
float   imu_pitch(void)  { return imu660rc_pitch; }
float   imu_roll(void)   { return imu660rc_roll; }
int16_t imu_gyro_x(void) { return fgx; }
int16_t imu_gyro_y(void) { return fgy; }
int16_t imu_gyro_z(void) { return fgz; }
int16_t imu_acc_x(void)  { return fax; }
int16_t imu_acc_y(void)  { return fay; }
int16_t imu_acc_z(void)  { return faz; }
