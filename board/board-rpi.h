#ifndef _BOARD_RPI_H
#define _BOARD_RPI_H


#define	PI_MODEL_A		 0
#define	PI_MODEL_B		 1
#define	PI_MODEL_AP		 2
#define	PI_MODEL_BP		 3
#define	PI_MODEL_2		 4
#define	PI_ALPHA		 5
#define	PI_MODEL_CM		 6
#define	PI_MODEL_07		 7
#define	PI_MODEL_3B		 8
#define	PI_MODEL_ZERO		 9
#define	PI_MODEL_CM3		10
#define	PI_MODEL_ZERO_W		12
#define	PI_MODEL_3BP 		13
#define	PI_MODEL_3AP 		14
#define	PI_MODEL_CM3P 		16
#define	PI_MODEL_4B 		17
#define	PI_MODEL_ZERO_2W	18
#define	PI_MODEL_400		19
#define	PI_MODEL_CM4		20

#define	GPIO_PERI_BASE_OLD  0x20000000
#define	GPIO_PERI_BASE_2835 0x3F000000
#define	GPIO_PERI_BASE_2711 0xFE000000

// GPPUD:
//	GPIO Pin pull up/down register

#define	GPPUD	37

/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0                57        /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1                58        /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2                59        /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3                60        /* Pin pull-up/down for pins 57:48 */



extern int RPI_GpioLayout (void);
extern int RPI_select();   //-1:当前板子不是树莓派 >=0:树莓派型号宏定义
extern int *RPI_get_pinToGpio();
extern int *RPI_get_physToGpio();
extern int *RPI_get_physToWpi();
extern char *RPI_get_physName();
extern char *RPI_get_alts();

extern int RPI_get_pin_count(); //返回当前板子有几个引脚

extern void RPI_get_model(int *model, int *rev);
extern void RPI_print_Version();
extern int RPI_get_GpioBase();
extern int RPI_get_GpioPupOffset();

#endif