#include "board-aw.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int pi_num_in_board = -1;
struct BOARD_ONE *PI_BOARD;


int pinToGpio_1B [64] =
{
    264, 
    263, 
    72, 261,
        262,
    73, 74,
    75, 
    267, 268,
        78,
    231, 
    232, 79,
    230, 229,
        233,	// 25, 26
    266,265,
    256,
    257,272,
    258,
    259, 271,
    260, 269,
        270,


  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 63
};

int physToGpio_1B [64] =
{
    -1,		// 0
    
    -1, -1,	// 1, 2
   264, -1,
   263, -1,
    72, 261,
    -1, 262,
    73, 74,
    75, -1,
    267, 268,
    -1, 78,
   231, -1,
   232, 79,
   230, 229,
    -1, 233,	// 25, 26
	266,265,
	256,-1,
	257,272,
	258,-1,
	259, 271,
	260, 269,
	-1, 270,


  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 56
  -1, -1, -1, -1,  // ... 63
} ;

int physToPin_1B [64] = 
{
  -1,           // 0
  -1, -1,       // 1, 2
   0, -1,
   1, -1,		
   2,  3,		//7, 8
  -1,  4,		
   5,  6,		//11, 12
   7, -1,		
   8,  9,		//15, 16
  -1, 10,		
  11, -1,		//19, 20
  12, 13,		
  14, 15,		//23, 24
  -1, 16,       // 25, 26
  17, 18,
  19, -1,
  20, 21,
  22, -1,
  23, 24,
  25, 26,
  -1, 28

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 56
  -1,  // ... 63
};

int PIN_MASK_1B[12][32] =  //[BANK]  [INDEX]
{
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
	{-1,-1,-1,-1,-1,-1,-1,-1, 8, 9,10,11,-1,-1,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
	{-1,-1,-1,-1,-1, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PJ
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PK
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PL
};
struct BOARD_ONE
{
    char *name;
    char *model_value;
    int men_size;

    int pin_count;    
    int *pinToGpio;
    int *physToGpio;
    int *physToPin;
    int *PIN_MASK;

    uint32_t GPIO_BASE;
};
                   

struct BOARD_ONE pi_boards[] = {
    {
        .name = "核桃派",
        .model_value = "01-1b",
        .men_size = 1024,

        .pin_count = 40,
        .pinToGpio = pinToGpio_1B,
        .physToGpio = physToGpio_1B,
        .physToPin = physToPin_1B,
        .PIN_MASK = (int *)PIN_MASK_1B,

        .GPIO_BASE = 0x0300B000,


    }

};
struct BOARD_ONE *PI_BOARD;
int AW_select(void)
{
    FILE *fp;
    char buffer[1024];
    char *model;
    int i;
    if (pi_num_in_board != -1)
        return pi_num_in_board;

    fp = fopen("/proc/device-tree/model", "r");
    if (fp == NULL)
    {
        printf("Failed to open /proc/device-tree/model \n");
        return -1;
    }

    fgets(buffer, 100, fp);
    fclose(fp);

    model = strtok(buffer, "\n");

    for (i = 0; i < (int)(sizeof(pi_boards) / sizeof(pi_boards[0])); i++)
    {
        if (strcmp(model, pi_boards[i].model_value) == 0)
        {
            printf("当前板子是: %s\n", pi_boards[i].name);
            pi_num_in_board = i;
            PI_BOARD = &pi_boards[i];
            return pi_num_in_board;
        }
    }
    return -1;
}
int *AW_get_pinToGpio()
{
    return PI_BOARD->pinToGpio;
}
int *AW_get_physToGpio()
{
    return PI_BOARD->physToGpio;
}
int AW_get_pin_count()
{
    return PI_BOARD->pin_count;
}
void AW_print_Version()
{
    printf("Board:\t%s\n\r\n",PI_BOARD->name);

}
int AW_get_GpioBase()
{
    return PI_BOARD->GPIO_BASE;

}
int AW_get_GpioPupOffset()
{
    return 0;
}
