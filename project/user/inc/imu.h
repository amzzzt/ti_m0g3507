/**
 * imu.h — IMU660RC 陀螺仪+RC封装
 *
 *   SPI0: SCK=B18 MOSI=B17 MISO=B19 CS=A2 INT2=B24
 */
#ifndef _imu_h_
#define _imu_h_
#include <stdint.h>

uint8_t imu_init(void);
void    imu_update(void);              /* 读四元数: yaw/pitch/roll自动更新 */

float   imu_yaw(void);                 /* 偏航角 °, DMP融合 */
float   imu_pitch(void);
float   imu_roll(void);
int16_t imu_gyro_x(void);              /* 原始角速度 */
int16_t imu_gyro_y(void);
int16_t imu_gyro_z(void);
int16_t imu_acc_x(void);
int16_t imu_acc_y(void);
int16_t imu_acc_z(void);

#endif
