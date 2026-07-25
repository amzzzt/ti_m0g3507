/**
 * imu.h — IMU660RA 陀螺仪/加速度计封装
 *
 *   SPI1: B23(SCK) B22(MOSI) B21(MISO) B19(CS)
 *   TIMA0 每 2ms 调 imu_update()
 */
#ifndef _imu_h_
#define _imu_h_

#include <stdint.h>

uint8_t imu_init(void);                 // 0=成功
void    imu_update(void);              // 读传感器 + 低通滤波 (每2ms)
int16_t imu_acc_x(void);               // 滤波后加速度
int16_t imu_acc_y(void);
int16_t imu_acc_z(void);
int16_t imu_gyro_x(void);              // 滤波后角速度
int16_t imu_gyro_y(void);
int16_t imu_gyro_z(void);

#endif
