#ifndef _BOARD_H
#define _BOARD_H
#include <stdint.h>
#include "board-aw.h"
#include "board-rpi.h"


#define PI_MODEL_NO_RASPBERRY 100


#define BOARD_IS_RPI        0
#define BOARD_IS_AW         1


extern int  Board_select(); //初始化，判断当前板子并设置各变量

extern void Board_get_model(int *model, int *rev);

//以下函数，都会在判断当前板子型号后，调用另外两个board头文件中的相应函数
extern int *Board_get_pinToGpio();
extern int *Board_get_physToGpio();
extern int *Board_get_physToWpi();
extern char **Board_get_physName();
extern int Board_get_pin_count(); //返回当前板子有几个引脚
extern char **Board_get_altsName();


void Board_print_Version();
extern int Board_get_GpioBase();


#endif
