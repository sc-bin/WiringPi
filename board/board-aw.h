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




extern char **AW_get_pwm_name();
extern char **AW_get_pwm_alts();

extern char **AW_get_uart_name();
extern char **AW_get_uart_alts();

extern char **AW_get_i2c_name();
extern char **AW_get_i2c_alts();

extern char **AW_get_spi_name();
extern char **AW_get_spi_alts();



#endif