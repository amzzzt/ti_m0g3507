/**
 * imu.c — IMU660RA 采集 + 低通滤波
 */
#include "zf_device_imu660ra.h"
#include "imu.h"

#define ALPHA  3      // α*10 = 3, 即 α=0.3

// 滤波状态 (α=0.3: filtered = (3*raw + 7*prev)/10)
static int16_t fax, fay, faz, fgx, fgy, fgz;

static int16_t lowpass(int16_t raw, int16_t *prev)
{
    int16_t f = (int16_t)(((int32_t)ALPHA * raw + (10 - ALPHA) * (*prev)) / 10);
    *prev = f;
    return f;
}

uint8_t imu_init(void)
{
    return imu660ra_init();   // 0=成功
}

void imu_update(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    lowpass(imu660ra_acc_x,  &fax);
    lowpass(imu660ra_acc_y,  &fay);
    lowpass(imu660ra_acc_z,  &faz);
    lowpass(imu660ra_gyro_x, &fgx);
    lowpass(imu660ra_gyro_y, &fgy);
    lowpass(imu660ra_gyro_z, &fgz);
}

int16_t imu_acc_x(void)  { return fax; }
int16_t imu_acc_y(void)  { return fay; }
int16_t imu_acc_z(void)  { return faz; }
int16_t imu_gyro_x(void) { return fgx; }
int16_t imu_gyro_y(void) { return fgy; }
int16_t imu_gyro_z(void) { return fgz; }
