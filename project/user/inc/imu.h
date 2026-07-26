/**
 * imu.h — IMU660RC 陀螺仪 (DMP 直接输出角度)
 *
 *   SPI0: B18(SCK) B17(MOSI) B19(MISO) A2(CS) B24(INT2)
 *   60Hz DMP 输出 yaw/pitch/roll
 */
#ifndef _imu_h_
#define _imu_h_

#include <stdint.h>

uint8_t imu_init(void);           /* 0=成功 */
void    imu_update(void);         /* 读 DMP, 更新角度 */
float   imu_yaw(void);            /* 偏航角 ° */
float   imu_pitch(void);          /* 俯仰角 ° */
float   imu_roll(void);           /* 横滚角 ° */

#endif
