#include "board-aw.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int pi_num_in_board = -1;
struct BOARD_ONE *PI_BOARD;

static char *alts [] =
{
  "IN", "OUT", "ALT2", "ALT3", "ALT4", "ALT5", "ALT6", "OFF"
} ;

int pinToGpio_1B [64] =
{
  73, 
  74,   75, 267,    268,    78, 79, 72, 264,    263,262,
  233,  231, 232,   230,    261, 262, -1, -1,   -1,  -1,
  256,  257, 258,   259,    260, 272, 271,269, 270, 266


  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 63
};
int physToWpi_1B [64] = 
{
	-1, 		  // 0
	-1, -1, 	  // 1, 2
	 8, -1,
	 9, -1, 	  
	 7,  15, 	  //7, 8
	-1,  16, 	  
	 0,  1, 	  //11, 12
	 2, -1, 	  
	 3,  4, 	  //15, 16
	-1, 5, 	  
	12, -1, 	  //19, 20
	13, 6, 	  
	14, 10, 	  //23, 24
	-1, 11, 	  // 25, 26
	30, 31,
	21, -1,
	22, 26,
	23, -1,
	24, 27,
    25, 28,
    -1, 29,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 56
	-1,  // ... 63

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


char *physName_1B [64] = 
{
  	   NULL,

 "    3.3V", "5V      ",
 "   SDA.1", "5V      ",
 "   SCL.1", "GND     ",
 "     PC8", "TXD.2   ",
 "     GND", "RXD.2   ",
 "     PC9", "PC10    ",
 "    PC11", "GND     ",
 "    PI11", "PI12    ",
 "    3.3V", "PC14    ",
 "  MOSI.1", "GND     ",
 "  MISO.1", "PC15    ",
 "  SCLK.1", "CS1.0   ",
 "     GND", "CS1.1   ",
 "   SDA.2", "SCL.2   ",
 "     PI0", "GND     ",
 "     PI1", "PI16    ",
 "     PI2", "GND     ",
 "     PI3", "PI15    ",
 "     PI4", "PI13    ",
 "     GND", "PI14    ",

       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       
};
int pwmToGpio_1B[] = { -1, 267, 268, 269, 270  };
struct BOARD_ONE
{
    char *name;
    char *model_value;
    int men_size;
    int pin_count;  

    int max_pwm;
    int *pwmToGpio;
    char **physName;
    int *pinToGpio;
    int *physToGpio;
    int *physToWpi;

    uint32_t mem_gpioA;
    uint32_t mem_gpioL;
    uint32_t mem_PWM;
};
                   

struct BOARD_ONE pi_boards[] = {
    {
        .name = "核桃派",
        .model_value = "01-1b",
        .men_size = 1024,
        .pin_count = 40,

        .max_pwm = 4,
        .pwmToGpio = pwmToGpio_1B,
        .physName = physName_1B,

        .pinToGpio = pinToGpio_1B,
        .physToGpio = physToGpio_1B,
        .physToWpi = physToWpi_1B,

        .mem_gpioA  = 0x0300B000,
        .mem_gpioL  = 0x07022000,
        .mem_PWM    = 0x0300A000,


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
            // printf("当前板子是: %s\n", pi_boards[i].name);
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
int *AW_get_physToWpi()
{
    return PI_BOARD->physToWpi;
}
int AW_get_pin_count()
{
    return PI_BOARD->pin_count;
}
char **AW_get_physName()
{
    // printf("AW_get_physName:%x\r\n", PI_BOARD->physName);
    return PI_BOARD->physName;
}
char **AW_get_alts()
{
    return alts;
}


void AW_print_Version()
{
    printf("Board:\t%s\n\r\n",PI_BOARD->name);

}
int AW_get_mem_gpioA()
{
    return PI_BOARD->mem_gpioA;
}
int AW_get_mem_gpioL()
{
    return PI_BOARD->mem_gpioL;
}

int AW_get_mem_PWM()
{
    return PI_BOARD->mem_PWM;
}