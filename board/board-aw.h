#ifndef _BOARD_AW_H
#define _BOARD_AW_H



int AW_select (void);   //-1:当前板子不是树莓派 >=0:树莓派型号宏定义
extern int *AW_get_pinToGpio();
extern int *AW_get_physToGpio();
extern int *AW_get_physToWpi();
extern char *AW_get_physName();
extern char *AW_get_alts();

extern int AW_get_pin_count(); //返回当前板子有几个引脚
extern void AW_print_Version();
extern int AW_get_GpioABase();
extern int AW_get_GpioLBase();
extern int AW_get_GpioPupOffset();
#endif