#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"

static int FLAG_Board_IS = -1;

int  Board_select()
{
    if (FLAG_Board_IS > -1)
        return FLAG_Board_IS;

    FLAG_Board_IS = AW_select();
    if (FLAG_Board_IS > -1)
    {
        FLAG_Board_IS = BOARD_IS_AW;
        return FLAG_Board_IS;
    }

    FLAG_Board_IS = RPI_select();
    if (FLAG_Board_IS > -1)
    {
        FLAG_Board_IS = BOARD_IS_RPI;
        return FLAG_Board_IS;
    }
    printf("error: you board not support!\r\n");
    exit(-1);
}
void Board_get_model(int *model, int *rev)
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_RPI)
        RPI_get_model(model, rev);
    else
        *model = PI_MODEL_NO_RASPBERRY;
}
int *Board_get_pinToGpio()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_pinToGpio();
    else
        return RPI_get_pinToGpio();
}
int *Board_get_physToGpio()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_physToGpio();
    else
        return RPI_get_physToGpio();
}
int *Board_get_physToWpi()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_physToWpi();
    else
        return RPI_get_physToWpi();
}
char **Board_get_physName()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_physName();
    else
        return RPI_get_physName();
}

int Board_get_pin_head_count()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_pin_head_count();
    else
        return RPI_get_pin_head_count();
}
int Board_get_pin_hw_count()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_pin_hw_count();
    else
        return -1;
}
char **Board_get_altsName()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_alts();
    else
        return RPI_get_alts();
    
}
void Board_print_Version()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        AW_print_Version();
    if (FLAG_Board_IS == BOARD_IS_RPI)
        RPI_print_Version();
}
int Board_get_GpioBase()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_mem_gpioA();
    else
        return RPI_get_GpioBase();
}

