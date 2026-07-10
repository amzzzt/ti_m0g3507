#ifndef _bsp_sr04_h_
#define _bsp_sr04_h_

#include <stdint.h>

extern volatile float sr04_measure;   // 测量结果 cm

void    sr04_init(void);
void    sr04_trigger(void);
uint8_t sr04_ready(void);
float   sr04_read(void);
uint8_t sr04_new_data(void);   // 有新滤波值, 读后自动清零

#endif
