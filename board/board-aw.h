#ifndef _BOARD_AW_H
#define _BOARD_AW_H



int AW_select (void);   //-1:当前板子不是树莓派 >=0:树莓派型号宏定义
extern int *AW_get_pinToGpio();
extern int *AW_get_physToGpio();
extern int *AW_get_physToWpi();
extern char **AW_get_physName();
extern char **AW_get_alts();


extern int AW_get_pin_head_count(); //返回主排针有几个引脚
extern int AW_get_pin_hw_count(); //返回板上连接到其他外设的io的数量
extern void AW_print_Version();


extern int AW_get_mem_gpioA();
extern int AW_get_mem_gpioL();

extern int AW_get_mem_PWM();
// extern void AW_get_max_HWpwm(); //最大的硬件pwm编号
// extern int *AW_get_pwmToGpio();

#endif