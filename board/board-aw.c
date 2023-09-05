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
  74,   75, 267,    268, 78,    79, 72, 264,    263,262,
  233,  231, 232,   230, 261,   262, -1, -1,   -1,  -1,
  256,  257, 258,   259, 260,   272, 271,269, 270, 266,
  265,   76,  77,

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 63
};
int physToWpi_1B [64] = 
{
	-1, 		  // 0
	-1, -1, 	  // 1, 2
	 8, -1,
	 9, -1, 	  
	 7, 15, 	  //7, 8
	-1, 16, 	  
	 0,  1, 	  //11, 12
	 2, -1, 	  
	 3,  4, 	  //15, 16
	-1,  5, 	  
	12, -1, 	  //19, 20
	13,  6, 	  
	14, 10, 	  //23, 24
	-1, 11, 	  // 25, 26
	30, 31,
	21, -1,
	22, 26,
	23, -1,
	24, 27,
    25, 28,
    -1, 29,
    32, 33,
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
    76, 77,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 56
  -1, -1, -1, -1,  // ... 63
} ;
#define COLOR_3V3_L     "\033[37;41m    3.3V\033[0m"
#define COLOR_5V_R      "\033[37;41m5V      \033[0m"
#define COLOR_GND_L     "\033[30;47m     GND\033[0m"
#define COLOR_GND_R     "\033[30;47mGND     \033[0m"

char *physName_1B [64] = 
{
  	   NULL,

COLOR_3V3_L, COLOR_5V_R,
 "   SDA.1", COLOR_5V_R,
 "   SCL.1", COLOR_GND_R,
 "     PC8", "TXD.2   ",
COLOR_GND_L, "RXD.2   ",
 "     PC9", "PC10    ",
 "    PC11", COLOR_GND_R,
 "    PI11", "PI12    ",
COLOR_3V3_L, "PC14    ",
 "  MOSI.1", COLOR_GND_R,
 "  MISO.1", "PC15    ",
 "  SCLK.1", "CS1.0   ",
COLOR_GND_L, "CS1.1   ",
 "   SDA.2", "SCL.2   ",
 "     PI0", COLOR_GND_R,
 "     PI1", "PI16    ",
 "     PI2", COLOR_GND_R,
 "     PI3", "PI15    ",
 "     PI4", "PI13    ",
COLOR_GND_L, "PI14    ",
 "     KEY", "LED     ",

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

// char *physName_1B [64] = 
// {
//   	   NULL,

// COLOR_3V3_L, COLOR_5V_R,
//  "   SDA.1", COLOR_5V_R,
//  "   SCL.1", COLOR_GND_R,
//  "     PC8", "TXD.2   ",
// COLOR_GND_L, "RXD.2   ",
//  "     PC9", "PC10    ",
//  "    PC11", COLOR_GND_R,
//  "    PWM1", "PWM2    ",
// COLOR_3V3_L, "PC14    ",
//  "  MOSI.1", COLOR_GND_R,
//  "  MISO.1", "PC15    ",
//  "  SCLK.1", "CS1.0   ",
// COLOR_GND_L, "CS1.1   ",
//  "   SDA.2", "SCL.2   ",
//  "     PI0", COLOR_GND_R,
//  "     PI1", "PI16    ",
//  "     PI2", COLOR_GND_R,
//  "     PI3", "PI15    ",
//  "     PI4", "TX4     ",
// COLOR_GND_L, "RX4     ",
//  "     KEY", "LED     ",

//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL,
//        NULL, NULL, 
// };


char *pwm_name_1b[]={
    " pwm1 "," pwm2 "," pwm3 "," pwm4 ",
};
int pwm_phys_1b[]={
    15,16,38,40
};
char *pwm_alt_1b[]={
    "alt5","alt5","alt5","alt5",
};


char *uart_name_1b[]={
    " tx-2 "," rx-2 "," tx-4 "," rx-4 ",
};
int uart_phys_1b[]={
    8,10,38,40
};
char *uart_alt_1b[]={
    "alt3","alt3","alt3","alt3",
};

char *i2c_name_1b[]={
    " SDA1 "," SCL1 "," SDA2 "," SCL2 ",
};
int i2c_phys_1b[]={
    3,5,27,28
};
char *i2c_alt_1b[]={
    "alt5","alt5","alt5","alt5",
};

char *spi_name_1b[]={
    "1-MOSI","1-MISO","1-CLK ", "1-CS0 ", "1-CS1 "
};
int spi_phys_1b[]={
    19,21,23,24,26
};
char *spi_alt_1b[]={
    "alt4","alt4","alt4","alt4","alt4"
};



int pwmToGpio_1B[] = { -1, 267, 268, 269, 270  };
struct BOARD_ONE
{
    char *name;
    char *model_value;
    int men_size;
    int pin_head_count;  
    int pin_hw_count;  

    int max_pwm;
    int *pwmToGpio;
    char **physName;
    int *pinToGpio;
    int *physToGpio;
    int *physToWpi;


    uint32_t mem_PWM;

    int pwm_count;
    char **pwmnames;
    int *pwmphys;
    char **pwmalts;

    
    int uart_count;
    char **uartnames;
    int *uartphys;
    char **uartalts;

    int i2c_count;
    char **i2cnames;
    int *i2cphys;
    char **i2calts;

    int spi_count;
    char **spinames;
    int *spiphys;
    char **spialts;



};
                   

struct BOARD_ONE pi_boards[] = {
    {
        .name = "核桃派",
        .model_value = "walnutpi-1b",
        .men_size = 1024,
        .pin_head_count = 40,
        .pin_hw_count = 2,
        .max_pwm = 4,
        .pwmToGpio = pwmToGpio_1B,
        .physName = physName_1B,

        .pinToGpio = pinToGpio_1B,
        .physToGpio = physToGpio_1B,
        .physToWpi = physToWpi_1B,


        .pwm_count = 4,
        .pwmnames = pwm_name_1b,
        .pwmphys = pwm_phys_1b,
        .pwmalts    =   pwm_alt_1b,
        
        .uart_count = 4,
        .uartnames = uart_name_1b,
        .uartphys = uart_phys_1b,
        .uartalts    =   uart_alt_1b,

        .i2c_count = 4,
        .i2cnames = i2c_name_1b,
        .i2cphys = i2c_phys_1b,
        .i2calts    =   i2c_alt_1b,

        .spi_count = 5,
        .spinames = spi_name_1b,
        .spiphys = spi_phys_1b,
        .spialts    =   spi_alt_1b,

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
int AW_get_pin_head_count()
{
    return PI_BOARD->pin_head_count;
}
int AW_get_pin_hw_count()
{
    return PI_BOARD->pin_hw_count;
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



char *pwmnNames[64]; 
char *pwmnalts[64]; 
char **AW_get_pwm_name()
{
    int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->pwm_count) && (PI_BOARD->pwmphys[p] ==i) )
        {
            pwmnNames[i] = PI_BOARD->pwmnames[p];
            p++;
        }
        else
            pwmnNames[i] = NULL;
    }
    return pwmnNames;
}
char **AW_get_pwm_alts()
{
     int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->pwm_count) && (PI_BOARD->pwmphys[p] ==i) )
        {
            pwmnalts[i] = PI_BOARD->pwmalts[p];
            p++;
        }
        else
            pwmnalts[i] = NULL;
    }
    return pwmnalts;

}




char *uartnNames[64]; 
char *uartnalts[64]; 
char **AW_get_uart_name()
{
    int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->uart_count) && (PI_BOARD->uartphys[p] ==i) )
        {
            uartnNames[i] = PI_BOARD->uartnames[p];
            p++;
        }
        else
            uartnNames[i] = NULL;
    }
    return uartnNames;
}
char **AW_get_uart_alts()
{
     int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->uart_count) && (PI_BOARD->uartphys[p] ==i) )
        {
            uartnalts[i] = PI_BOARD->uartalts[p];
            p++;
        }
        else
            uartnalts[i] = NULL;
    }
    return uartnalts;

}


char *i2cnNames[64]; 
char *i2cnalts[64]; 
char **AW_get_i2c_name()
{
    int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->i2c_count) && (PI_BOARD->i2cphys[p] ==i) )
        {
            i2cnNames[i] = PI_BOARD->i2cnames[p];
            p++;
        }
        else
            i2cnNames[i] = NULL;
    }
    return i2cnNames;
}
char **AW_get_i2c_alts()
{
     int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->i2c_count) && (PI_BOARD->i2cphys[p] ==i) )
        {
            i2cnalts[i] = PI_BOARD->i2calts[p];
            p++;
        }
        else
            i2cnalts[i] = NULL;
    }
    return i2cnalts;

}


char *spinNames[64]; 
char *spinalts[64]; 
char **AW_get_spi_name()
{
    int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->spi_count) && (PI_BOARD->spiphys[p] ==i) )
        {
            spinNames[i] = PI_BOARD->spinames[p];
            p++;
        }
        else
            spinNames[i] = NULL;
    }
    return spinNames;
}
char **AW_get_spi_alts()
{
     int p=0;
    for( int i = 0; i<64; i++ )
    {
        if( (p < PI_BOARD->spi_count) && (PI_BOARD->spiphys[p] ==i) )
        {
            spinalts[i] = PI_BOARD->spialts[p];
            p++;
        }
        else
            spinalts[i] = NULL;
    }
    return spinalts;

}
