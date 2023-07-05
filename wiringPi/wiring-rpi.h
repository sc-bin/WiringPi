#ifndef _WIRINGRPI_H
#define _WIRINGRPI_H


#define	BCM_PASSWORD		0x5A000000


//	Clock regsiter offsets

#define	PWMCLK_CNTL	40
#define	PWMCLK_DIV	41

#define	PWM0_MS_MODE    0x0080  // Run in MS mode
#define	PWM0_USEFIFO    0x0020  // Data from FIFO
#define	PWM0_REVPOLAR   0x0010  // Reverse polarity
#define	PWM0_OFFSTATE   0x0008  // Ouput Off state
#define	PWM0_REPEATFF   0x0004  // Repeat last value if FIFO empty
#define	PWM0_SERIAL     0x0002  // Run in serial mode
#define	PWM0_ENABLE     0x0001  // Channel Enable

#define	PWM1_MS_MODE    0x8000  // Run in MS mode
#define	PWM1_USEFIFO    0x2000  // Data from FIFO
#define	PWM1_REVPOLAR   0x1000  // Reverse polarity
#define	PWM1_OFFSTATE   0x0800  // Ouput Off state
#define	PWM1_REPEATFF   0x0400  // Repeat last value if FIFO empty
#define	PWM1_SERIAL     0x0200  // Run in serial mode
#define	PWM1_ENABLE     0x0100  // Channel Enable

// Timer
//	Word offsets
#define	TIMER_LOAD	(0x400 >> 2)
#define	TIMER_VALUE	(0x404 >> 2)
#define	TIMER_CONTROL	(0x408 >> 2)
#define	TIMER_IRQ_CLR	(0x40C >> 2)
#define	TIMER_IRQ_RAW	(0x410 >> 2)
#define	TIMER_IRQ_MASK	(0x414 >> 2)
#define	TIMER_RELOAD	(0x418 >> 2)
#define	TIMER_PRE_DIV	(0x41C >> 2)
#define	TIMER_COUNTER	(0x420 >> 2)

// PWM
//	Word offsets into the PWM control region

#define	PWM_CONTROL 0
#define	PWM_STATUS  1
#define	PWM0_RANGE  4
#define	PWM0_DATA   5
#define	PWM1_RANGE  8
#define	PWM1_DATA   9


#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010


extern int BCM_setup();
extern void usingGpioMemCheck (const char *what);
extern int BCM_gpio_read(int gpio_num);
extern void BCM_gpio_write(int gpio_num, int value);

extern void BCM_pwmWrite(int pin, int value);
extern void BCM_pwmSetMode (int mode);
extern void BCM_pwmSetRange (unsigned int range);
extern void BCM_pwmSetClock (int divisor);
extern void BCM_gpioClockSet (int pin, int freq);
extern void BCM_pullUpDnControl (int pin, int pud);
extern void BCM_setPadDrive (int group, int value);
extern int BCM_getAlt (int pin);
extern void BCM_pinModeAlt (int gpio_num, int mode);
extern void bcm_pinMode (int gpio_num, int mode);


#endif