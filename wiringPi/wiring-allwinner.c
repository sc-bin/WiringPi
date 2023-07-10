

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <limits.h>
#include "softPwm.h"
#include "softTone.h"

#include "wiring-allwinner.h"
#include "../board/board.h"
#include "wiringPi.h"

int pwmmode = 1;
uint32_t MAP_MASK = 0Xfff;

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

static int fd_mem;
volatile uint32_t *mmap_gpio;
volatile uint32_t *mmap_pwm;

void write_mem_pwm(unsigned int val, unsigned int addr)
{
    unsigned int mmap_base = (addr & ~MAP_MASK);
    unsigned int mmap_seek = ((addr - mmap_base) >> 2);

    *(mmap_pwm + mmap_seek) = val;
}
unsigned int read_mem_pwm(unsigned int addr)
{

    uint32_t val = 0;
    uint32_t mmap_base = (addr & ~MAP_MASK);
    uint32_t mmap_seek = ((addr - mmap_base) >> 2);
    val = *(mmap_pwm + mmap_seek);
    return val;
}
#define PWM_ADDR (unsigned int)0x0300A000
#define PCGR01 (PWM_ADDR + 0X20)
#define PCGR23 (PWM_ADDR + 0X24)
#define PCGR45 (PWM_ADDR + 0X28)

#define PDZCR01 (PWM_ADDR + 0X30)
#define PDZCR23 (PWM_ADDR + 0X34)
#define PDZCR45 (PWM_ADDR + 0X38)

#define PER (PWM_ADDR + 0X40)

#define PWM_PCR_BASE (PWM_ADDR + 0x60)
#define PWM_PCR_STEP 0X20

#define PCR1 (PWM_ADDR + 0x60 + (0x20 * 1))
#define PCR2 (PWM_ADDR + 0x60 + (0x20 * 2))
#define PCR3 (PWM_ADDR + 0x60 + (0x20 * 3))
#define PCR4 (PWM_ADDR + 0x60 + (0x20 * 4))

#define PWM_PPR_BASE (PWM_ADDR + 0x60 + 0x04)
#define PWM_PPR_STEP 0X20
#define PPR1 (PWM_ADDR + 0x60 + 0x04 + (0x20 * 1))
#define PPR2 (PWM_ADDR + 0x60 + 0x04 + (0x20 * 2))
#define PPR3 (PWM_ADDR + 0x60 + 0x04 + (0x20 * 3))
#define PPR4 (PWM_ADDR + 0x60 + 0x04 + (0x20 * 4))

void reg_print()
{
    printf("PCGR01=%x\r\n", read_mem_pwm(PCGR01));
    printf("PCGR23=%x\r\n", read_mem_pwm(PCGR23));
    printf("PCGR45=%x\r\n", read_mem_pwm(PCGR45));

    printf("PDZCR01=%x\r\n", read_mem_pwm(PDZCR01));
    printf("PDZCR23=%x\r\n", read_mem_pwm(PDZCR23));
    printf("PDZCR45=%x\r\n", read_mem_pwm(PDZCR45));

    printf("PER=%x\r\n", read_mem_pwm(PER));

    printf("PCR1=%x\r\n", read_mem_pwm(PCR1));
    printf("PCR2=%x\r\n", read_mem_pwm(PCR2));
    printf("PCR3=%x\r\n", read_mem_pwm(PCR3));
    printf("PCR4=%x\r\n", read_mem_pwm(PCR4));

    printf("PPR1=%x\r\n", read_mem_pwm(PPR1));
    printf("PPR2=%x\r\n", read_mem_pwm(PPR2));
    printf("PPR3=%x\r\n", read_mem_pwm(PPR3));
    printf("PPR4=%x\r\n", read_mem_pwm(PPR4));
}
void sunxi_pwm_init()
{
    // 4和5共用时钟源，而5被用作phy的时钟，所以就不改了
    //  设置时钟源为apb100M，分频系数1
    // write_mem_pwm(0x90, PCGR01);
    // write_mem_pwm(0x90, PCGR23);
    // 设置时钟源为osc24M，分频系数1
    // write_mem_pwm(0x10, PCGR01);
    // write_mem_pwm(0x10, PCGR23);

    // 设置activer level为高电平
    write_mem_pwm(0x100, PCR1);
    write_mem_pwm(0x100, PCR2);
    write_mem_pwm(0x100, PCR3);
    write_mem_pwm(0x100, PCR4);
}

void pwm_set_div(int pwm_num, int div)
{
    uint32_t reg, value = 0;
    if (div < 0 || div > 256)
    {
        printf("div分频系数错误\r\n");
        exit(-1);
    }
    while (div > 1)
    {
        value++;
        div /= 2;
    }

    switch (pwm_num)
    {
    case 1:
        reg = PCGR01;
        break;
    case 2:
    case 3:
        reg = PCGR23;
        break;
    default:
        if (value > 0)
        {
            printf("该引脚不可设置分频\r\n");
            exit(-1);
        }
    }

    value += 0x10;
    write_mem_pwm(value, reg);
}
void sunxi_pwm_set_period_cycle(int num, uint16_t period, uint16_t act)
{
    uint32_t tmp;
    tmp = period << 16;
    tmp += act;
    write_mem_pwm(tmp, PWM_PPR_BASE + (PWM_PPR_STEP * num));
}

int num2pwm[5] = {-1, 267, 268, 269, 270};
int gpio2pwm(int gpio_num)
{
    // printf("gpio_num=%d\r\n", gpio_num);
    for (int i = 0; i < 5; i++)
    {
        if (num2pwm[i] == gpio_num)
            return i;
    }
    return -1;
}
// s - ms - us -ns
// 24M下，最小计数单元是 41ns
// 16位最大值是65565，即分频1下，最小单元是41ns，最大周期是2.6ms，即384hz
// 分频2，最小单元是82ns，最大周期是5.2ms，即192hz
// 分频4，最小单元是164ns，最大是96hz
// 分频8，最小单元是328ns，最大是48hz
// 分频16，最小单元是656ns，最大是24hz
// 分频32，最小单元是1312ns，最大是12hz
// 分频64，最小单元是1312ns，最大是12hz

void sunxi_pwmWrite(int gpio_num, int value, int freq)
{
    // printf("sunxi_pwmWrite\r\n");
    // printf("gpio_num = %d\r\n", gpio_num);
    // printf("value = %d\r\n", value);
    // 100M = 10ns
    int pwm_num = gpio2pwm(gpio_num);
    if (pwm_num < 0)
    {
        printf("引脚错误\r\n");
        exit(-1);
    }

    if (freq > (FREQ_CLOCK / 1000))
    {
        printf("频率过高。输出不了1k分辨率\r\n");
        exit(-1);
    }
    if (value < 0 || value > 1000)
    {
        printf("占空比错误\r\n");
        exit(-1);
    }

    int time_period = NS_1s / freq; // 计算周期ns
    // printf("周期=%dns\r\n", time_period);

    int div = 1;
    int cycle1;
    int cycle_period;
    int cycle_k1;
    if (pwm_num == 4)
    {
        cycle1 = NS_1s / FREQ_CLOCK * div;

        cycle_period = time_period / cycle1;
        cycle_k1 = time_period / 1000 * value / cycle1;
    }
    else
    {
        do
        {
            cycle1 = NS_1s / FREQ_CLOCK * div;

            cycle_period = time_period / cycle1;

            if (cycle_period > 0xffff)
                continue;
            cycle_k1 = time_period / 1000 * value / cycle1;
            if (cycle_k1 > 0xffff)
                continue;
            break;

        } while ((div *= 2) <= 256);
    }

    SUNXI_pin_set_alt(gpio_num, 5);
    pwm_set_div(pwm_num, div);
    sunxi_pwm_set_period_cycle(pwm_num, cycle_period, cycle_k1);
    write_mem_pwm(0x20, PER);
    write_mem_pwm(0xff, PER);
}

void sunxi_pwmwrite_time(int gpio_num, int high_time, int period_time)
{


    int pwm_num = gpio2pwm(gpio_num);
    if (pwm_num < 0)
    {
        printf("引脚错误\r\n");
        exit(-1);
    }

    if (high_time > 50000 || high_time > 5000)
    {
        printf("时间错误\r\n");
        exit(-1);
    }
    if (pwm_num == 4)
    {
        if (high_time > 2500 || high_time > 2500)
        {
            printf("pwm4不允许这么长的时间\r\n");
            exit(-1);
        }
    }
    high_time *= 1000;
    period_time *= 1000;
    int div = 1;
    int cycle1;
    int cycle_period;
    int cycle_k1;
    if (pwm_num == 4)
    {
        cycle1 = NS_1s / FREQ_CLOCK * div;
        cycle_period = period_time / cycle1;
        cycle_k1 = high_time / cycle1;
    }
    else
    {
        do
        {
            cycle1 = NS_1s / FREQ_CLOCK * div;
            cycle_period = period_time / cycle1;

            if (cycle_period > 0xffff)
                continue;

            cycle_k1 = high_time / cycle1;
            if (cycle_k1 > 0xffff)
                continue;
            break;

        } while ((div *= 2) <= 256);
    }
    // printf("cycle_period=%d\r\n", cycle_period);
    // printf("cycle_k1=%d\r\n", cycle_k1);
    SUNXI_pin_set_alt(gpio_num, 5);
    pwm_set_div(pwm_num, div);
    sunxi_pwm_set_period_cycle(pwm_num, cycle_period, cycle_k1);
    write_mem_pwm(0x20, PER);
    write_mem_pwm(0xff, PER);
}

void SUNXI_init()
{

    AW_select();
    fd_mem = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (fd_mem < 0)
    {
        printf("Failed to open /dev/mem\r\n");
        exit(-1);
    }

    mmap_gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, AW_get_mem_gpioA());
    mmap_pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, AW_get_mem_PWM());
    wiringPiMode = WPI_MODE_PINS;

    sunxi_pwm_init();
}

const char *int2bin(uint32_t param)
{
    int bits = sizeof(uint32_t) * CHAR_BIT;
    static char buffer[sizeof(uint32_t) * CHAR_BIT + 1];
    char chars[2] = {'0', '1'};
    int i, j, offset;
    for (i = 0; i < bits; i++)
    {
        j = bits - i - 1;
        offset = (param & (1 << j)) >> j;
        buffer[i] = chars[offset];
    }
    buffer[bits] = '\0';
    return buffer;
}
void write_mem_gpio(unsigned int val, unsigned int addr)
{
    unsigned int mmap_base = (addr & ~MAP_MASK);
    unsigned int mmap_seek = ((addr - mmap_base) >> 2);

    *(mmap_gpio + mmap_seek) = val;
}

unsigned int read_mem_gpio(unsigned int addr)
{

    uint32_t val = 0;
    uint32_t mmap_base = (addr & ~MAP_MASK);
    uint32_t mmap_seek = ((addr - mmap_base) >> 2);
    val = *(mmap_gpio + mmap_seek);
    return val;
}

void SUNXI_gpio_write(int pin, int value)
{
    unsigned int bank = pin >> 5;
    unsigned int index = pin - (bank << 5);
    unsigned int phyaddr = 0;

    unsigned int regval = 0;

    if (bank == 11)
        phyaddr = AW_get_mem_gpioL() + 0x10;
    else
        phyaddr = AW_get_mem_gpioA() + (bank * 36) + 0x10;

    regval = read_mem_gpio(phyaddr);
    if (wiringPiDebug)
        printf("befor write reg val: 0x%x,index:%d\n", regval, index);
    if (0 == value)
    {
        regval &= ~(1 << index);
        write_mem_gpio(regval, phyaddr);
        regval = read_mem_gpio(phyaddr);
        if (wiringPiDebug)
            printf("LOW val set over reg val: 0x%x\n", regval);
    }
    else
    {
        regval |= (1 << index);
        write_mem_gpio(regval, phyaddr);
        regval = read_mem_gpio(phyaddr);
        if (wiringPiDebug)
            printf("HIGH val set over reg val: 0x%x\n", regval);
    }
}

int SUNXI_gpio_read(int gpio_num)
{

    int bank = gpio_num >> 5;
    int index = gpio_num - (bank << 5);
    int val;

    unsigned int phyaddr;

    if (bank == 11)
        phyaddr = AW_get_mem_gpioL() + 0x10;
    else
        phyaddr = AW_get_mem_gpioA() + (bank * 36) + 0x10;

    if (read_mem_gpio(phyaddr) & GPIO_BIT(index)) /* Input */
        val = (read_mem_gpio(phyaddr) & GPIO_BIT(index)) ? 1 : 0;
    else /* Ouput */
        val = (read_mem_gpio(phyaddr) & GPIO_BIT(index)) ? 1 : 0;
    return val;

    return 0;
}
void SUNXI_pin_set_mode(int gpio_num, int mode)
{
    unsigned int regval = 0;
    unsigned int bank = gpio_num >> 5;
    unsigned int index = gpio_num - (bank << 5);
    unsigned int phyaddr = 0;

    int offset = ((index - ((index >> 3) << 3)) << 2);

    if (bank == 11)
    {
        phyaddr = AW_get_mem_gpioL() + ((index >> 3) << 2);
    }
    else
        phyaddr = AW_get_mem_gpioA() + (bank * 36) + ((index >> 3) << 2);

    regval = read_mem_gpio(phyaddr);
    if (wiringPiDebug)
        printf("PinMode: pin:%d,mode:%d\n", gpio_num, mode);

    if (-1 == gpio_num)
    {
        printf("[%s:L%d] the pin:%d is invaild,please check it over!\n",
               __func__, __LINE__, gpio_num);
        return;
    }

    if (mode == INPUT)
    {
        regval &= ~(7 << offset);
        write_mem_gpio(regval, phyaddr);
        regval = read_mem_gpio(phyaddr);
        SUNXI_gpio_set_PullUpDn(gpio_num, PUD_OFF);
        return;
    }
    else if (mode == OUTPUT)
    {
        regval &= ~(7 << offset);
        regval |= (1 << offset);

        write_mem_gpio(regval, phyaddr);
        regval = read_mem_gpio(phyaddr);

        return;
    }
    else
        return;
}
int SUNXI_pin_get_alt(int gpio_num)
{
    unsigned int regval = 0;
    unsigned int bank = gpio_num >> 5;
    unsigned int index = gpio_num - (bank << 5);
    unsigned int phyaddr = 0;
    unsigned char mode = -1;
    uint32_t GPIOA_BASE = AW_get_mem_gpioA();
    uint32_t GPIOL_BASE = AW_get_mem_gpioL();

    int offset = ((index - ((index >> 3) << 3)) << 2);
    // printf("\r\nbank=%d\r\n",bank);
    if (bank == 11)
    {
        phyaddr = GPIOL_BASE + ((index >> 3) << 2);
    }
    else
        phyaddr = GPIOA_BASE + (bank * 36) + ((index >> 3) << 2);

    // printf("phyaddr=%x\r\n", phyaddr);
    regval = read_mem_gpio(phyaddr);
    // printf("egval=%x\r\n", regval);

    mode = (regval >> offset) & 7;

    return mode;
}
void SUNXI_gpio_set_PullUpDn(int gpio_num, int pud)
{
    unsigned int regval = 0;
    unsigned int bank = gpio_num >> 5;
    unsigned int index = gpio_num - (bank << 5);
    unsigned int phyaddr = 0;
    unsigned int bit_value = -1, bit_enable = 0;
    unsigned int offset;
    uint32_t GPIOA_BASE = AW_get_mem_gpioA();
    uint32_t GPIOL_BASE = AW_get_mem_gpioL();

    unsigned int pullOffset = 0x1C;
    switch (pud)
    {
    case PUD_OFF:
        bit_value = SUNXI_PUD_OFF;
        break;
    case PUD_UP:
        SUNXI_pin_set_mode(gpio_num, 0);
        bit_value = SUNXI_PUD_UP;
        break;
    case PUD_DOWN:
        SUNXI_pin_set_mode(gpio_num, 0);
        bit_value = SUNXI_PUD_DOWN;
        break;
    default:
        printf("Unknow pull mode\n");
        return;
    }
    offset = ((index - ((index >> 4) << 4)) << 1);
    pullOffset = 0x1C;

    if (bank == 11)
    {
        phyaddr = pullOffset + GPIOL_BASE + ((index >> 4) << 2);
    }
    else
        phyaddr = pullOffset + GPIOA_BASE + (bank * 36) + ((index >> 4) << 2);

    regval = read_mem_gpio(phyaddr);
    if (wiringPiDebug)
        printf("read val(%#x) from register[%#x]\n", regval, phyaddr);

    /* clear bit */
    regval &= ~(3 << offset);

    /* bit write enable*/
    regval |= bit_enable;

    /* set bit */
    regval |= (bit_value & 3) << offset;

    write_mem_gpio(regval, phyaddr);
    regval = read_mem_gpio(phyaddr);
}
void SUNXI_pin_set_alt(int gpio_num, int mode)
{

    unsigned int regval = 0;
    unsigned int bank = gpio_num >> 5;
    unsigned int index = gpio_num - (bank << 5);
    unsigned int phyaddr = 0;
    int offset = ((index - ((index >> 3) << 3)) << 2);
    uint32_t GPIOA_BASE = AW_get_mem_gpioA();
    uint32_t GPIOL_BASE = AW_get_mem_gpioL();

    if (bank == 11)
        phyaddr = GPIOL_BASE + ((index >> 3) << 2);
    else
        phyaddr = GPIOA_BASE + (bank * 36) + ((index >> 3) << 2);

    regval = read_mem_gpio(phyaddr);
    regval &= ~(7 << offset);
    regval |= (mode << offset);
    write_mem_gpio(regval, phyaddr);
}