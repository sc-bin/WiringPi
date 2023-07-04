#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"

static int FLAG_Board_IS = -1;

void Board_select()
{
    if (FLAG_Board_IS > -1)
        return;

    FLAG_Board_IS = AW_select();
    if (FLAG_Board_IS > -1)
    {
        FLAG_Board_IS = BOARD_IS_AW;
        return;
    }

    FLAG_Board_IS = RPI_select();
    if (FLAG_Board_IS > -1)
    {
        FLAG_Board_IS = BOARD_IS_RPI;
        return;
    }
    printf("error: you board not support!\r\n");
    exit(-1);
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
int Board_get_pin_count()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_pin_count();
    else
        return RPI_get_pin_count();
    
}
void Board_print_Version()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_print_Version();
    if (FLAG_Board_IS == BOARD_IS_RPI)
        return RPI_print_Version();
}
int Board_get_GpioBase()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_GpioBase();
    else
        return RPI_get_GpioBase();
}
int Board_get_GpioPupOffset()
{
    Board_select();
    if (FLAG_Board_IS == BOARD_IS_AW)
        return AW_get_GpioPupOffset();
    else
        return RPI_get_GpioPupOffset();
}