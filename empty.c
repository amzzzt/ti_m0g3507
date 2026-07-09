#include "ti_msp_dl_config.h"
#include "servo.h"

int main(void)
{
    SYSCFG_DL_init();
    servo_init();

    while (1) {
        servo_sweep();
    }
}
