/**
 * menu.h — TFT180 多级菜单框架
 *
 * KEY1(A30)=确定, KEY2(A31)=上, KEY3(B0)=下
 */

#ifndef _menu_h_
#define _menu_h_

#include <stdint.h>

void    menu_init(void);
void    menu_run(void);
uint8_t menu_is_active(void);   // 1=菜单中(跳过波形), 0=正常显示

#endif
