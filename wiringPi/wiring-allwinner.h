#ifndef _WIRING_ALLWINNER_H
#define _WIRING_ALLWINNER_H
#include "wiringPi.h"

#define SUNXI_PUD_OFF   0
#define SUNXI_PUD_UP    1
#define SUNXI_PUD_DOWN  2

//sunxi_pwm
#define SUNXI_PWM_BASE (0x01c21400)
#define SUNXI_PWM_CTRL_REG  (SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD  (SUNXI_PWM_BASE + 0x4)
#define SUNXI_PWM_CH1_PERIOD  (SUNXI_PWM_BASE + 0x8)

#define SUNXI_PWM_CH0_EN   (1 << 4)
#define SUNXI_PWM_CH0_ACT_STA  (1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING (1 << 6)
#define SUNXI_PWM_CH0_MS_MODE  (1 << 7) //pulse mode
#define SUNXI_PWM_CH0_PUL_START  (1 << 8)

#define PWM_CLK_DIV_120  0
#define PWM_CLK_DIV_180  1
#define PWM_CLK_DIV_240  2
#define PWM_CLK_DIV_360  3
#define PWM_CLK_DIV_480  4
#define PWM_CLK_DIV_12K  8
#define PWM_CLK_DIV_24K  9
#define PWM_CLK_DIV_36K  10
#define PWM_CLK_DIV_48K  11
#define PWM_CLK_DIV_72K  12

#define GPIO_BIT(x)                        (1UL << (x))


extern void SUNXI_init();

extern void SUNXI_pin_set_mode (int gpio_num, int mode);
extern void SUNXI_pin_set_alt (int gpio_num, int mode);
extern int SUNXI_pin_get_alt (int gpio_num);

extern int SUNXI_gpio_read(int gpio_num);
extern void SUNXI_gpio_write(int gpio_num, int value);
extern void SUNXI_gpio_set_PullUpDn (int gpio_num, int pud);

// extern void BCM_pwmWrite(int gpio_num, int value);
// extern void BCM_pwmSetMode (int mode);
// extern void BCM_pwmSetRange (unsigned int range);
// extern void BCM_pwmSetClock (int divisor);

// extern void sunxi_pwm_set_range(int range);
// extern void sunxi_pwm_write(int gpio_num, int value);

extern void print_pwm_reg(void);
extern void sunxi_pwm_set_enable(int en);
extern void sunxi_pwm_set_mode(int mode);
extern void sunxi_pwm_set_clk(int clk);
extern int sunxi_pwm_get_period(void);
extern void sunxi_pwm_set_period(int period_cys);
extern int sunxi_pwm_get_act(void);
extern void sunxi_pwm_set_act(int act_cys);
extern void sunxi_pwmWrite(int gpio_num, int value);


#endif