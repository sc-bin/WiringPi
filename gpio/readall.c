/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2018 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://github.com/WiringPi/WiringPi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>
#include "../board/board.h"

extern int wpMode;

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (1 == 2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal(void)
{
  int pin;

  printf("+------+---------+--------+\n");
  printf("|  Pin | Digital | Analog |\n");
  printf("+------+---------+--------+\n");

  for (pin = wiringPiNodes->pinBase; pin <= wiringPiNodes->pinMax; ++pin)
    printf("| %4d |  %4d   |  %4d  |\n", pin, digitalRead(pin), analogRead(pin));

  printf("+------+---------+--------+\n");
}

/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| BCM | wPi |   Name  | Mode | Val| Physical |Val | Mode | Name    | wPi | BCM |
 *********************************************************************************
 */

static void readallPhys(int physPin)
{
  int pin;
  int *phy2gpio = Board_get_physToGpio();
  int *phy2wpi = Board_get_physToWpi();
  char **physname = Board_get_physName();
  if (phy2gpio[physPin] < 0)
    printf(" |     |    ");
  else
    printf(" | %3d | %3d", phy2gpio[physPin], phy2wpi[physPin]);

  printf(" | %s", physname[physPin]);

  if (phy2gpio[physPin] < 0)
    printf(" |      |  ");
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio(physPin);
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin;
    else
      pin = phy2wpi[physPin];

    // printf ("\r\n getAlt=%d\r\n", getAlt (pin)) ;
    printf(" | %4s", Board_get_altsName()[getAlt(pin)]);
    printf(" | %d", digitalRead(pin));
  }

  // Pin numbers:

  printf(" | %2d", physPin);
  ++physPin;
  printf(" || %-2d", physPin);

  // Same, reversed
  if (phy2gpio[physPin] < 0)
    printf(" |   |     ");
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio(physPin);
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin;
    else
      pin = phy2wpi[physPin];

    printf(" | %d", digitalRead(pin));
    printf(" | %-4s", Board_get_altsName()[getAlt(pin)]);
  }

  printf(" | %-5s", physname[physPin]);

  if (phy2gpio[physPin] < 0)
    printf(" |     |    ");
  else
    printf(" | %-3d | %-3d", phy2wpi[physPin], phy2gpio[physPin]);

  printf(" |\n");
}

/*
 * allReadall:
 *	Read all the pins regardless of the model. Primarily of use for
 *	the compute module, but handy for other fiddling...
 *********************************************************************************
 */

static void allReadall(void)
{

  printf("+-----+------+-------+      +-----+------+-------+\n");
  printf("| Pin | Mode | Value |      | Pin | Mode | Value |\n");
  printf("+-----+------+-------+      +-----+------+-------+\n");

  int pin_count = Board_get_pin_count() / 2;
  int *gpio = Board_get_physToGpio();
  for (int i = 0; i < pin_count; ++i)
  {
    printf("| %3d ", i);
    printf("| %-4s ", Board_get_altsName()[getAlt(i)]);
    printf("| %s  ", digitalRead(gpio[i]) == HIGH ? "High" : "Low ");
    printf("|      ");
    printf("| %3d ", i + pin_count);
    printf("| %-4s ", Board_get_altsName()[getAlt(i + pin_count)]);
    printf("| %s  ", digitalRead(gpio[i + pin_count]) == HIGH ? "High" : "Low ");
    printf("|\n");
  }

  printf("+-----+------+-------+      +-----+------+-------+\n");
}

/*
 * abReadall:
 *	Read all the pins on the model A or B.
 *********************************************************************************
 */

void abReadall(int model, int rev)
{
  int pin;
  char *type;

  if (model == PI_MODEL_A)
    type = " A";
  else if (rev == PI_VERSION_2)
    type = "B2";
  else
    type = "B1";

  printf(" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type);
  printf(" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n");
  printf(" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n");
  for (pin = 1; pin <= 26; pin += 2)
    readallPhys(pin);

  if (rev == PI_VERSION_2) // B version 2
  {
    printf(" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n");
    for (pin = 51; pin <= 54; pin += 2)
      readallPhys(pin);
  }

  printf(" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n");
  printf(" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n");
  printf(" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type);
}

/*
 * piPlusReadall:
 *	Read all the pins on the model A+ or the B+ or actually, all 40-pin Pi's
 *********************************************************************************
 */

static void plus2header(int model)
{
  /**/ if (model == PI_MODEL_AP)
    printf(" +-----+-----+---------+------+---+---Pi A+--+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_BP)
    printf(" +-----+-----+---------+------+---+---Pi B+--+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_ZERO)
    printf(" +-----+-----+---------+------+---+-Pi Zero--+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_ZERO_W)
    printf(" +-----+-----+---------+------+---+-Pi ZeroW-+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_ZERO_2W)
    printf(" +-----+-----+---------+------+---+Pi Zero 2W+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_2)
    printf(" +-----+-----+---------+------+---+---Pi 2---+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_3B)
    printf(" +-----+-----+---------+------+---+---Pi 3B--+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_3BP)
    printf(" +-----+-----+---------+------+---+---Pi 3B+-+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_3AP)
    printf(" +-----+-----+---------+------+---+---Pi 3A+-+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_4B)
    printf(" +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+\n");
  else if (model == PI_MODEL_400)
    printf(" +-----+-----+---------+------+---+---Pi 400-+---+------+---------+-----+-----+\n");
  else
    printf(" +-----+-----+---------+------+---+---Pi ?---+---+------+---------+-----+-----+\n");
}

static void piPlusReadall(int model)
{
  int pin;

  plus2header(model);
  printf(" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n");
  printf(" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n");
  for (pin = 1; pin <= 40; pin += 2)
    readallPhys(pin);
  printf(" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n");
  printf(" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n");

  plus2header(model);
}
static void aw_readall()
{
  int pin;
  printf(" +------+-----+----------+------+---+----++----+---+------+----------+-----+------+\n");
  printf(" | GPIO | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | GPIO |\n");
  printf(" +------+-----+----------+------+---+----++----+---+------+----------+-----+------+\n");

  for (pin = 1; pin <= Board_get_pin_count(); pin += 2)
    readallPhys(pin);

  printf(" +------+-----+----------+------+---+----++----+---+------+----------+-----+------+\n");
  printf(" | GPIO | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | GPIO |\n");
  printf(" +------+-----+----------+------+---+----++----+---+------+----------+-----+------+\n");
}

/*
 * doReadall:
 *	Generic read all pins called from main program. Works out the Pi type
 *	and calls the appropriate function.
 *********************************************************************************
 */

void doReadall(void)
{
  int model, rev;

  if (wiringPiNodes != NULL) // External readall
  {
    doReadallExternal();
    return;
  }

  Board_get_model(&model, &rev);
  /**/ if ((model == PI_MODEL_A) || (model == PI_MODEL_B))
    abReadall(model, rev);
  else if ((model == PI_MODEL_BP) || (model == PI_MODEL_AP) ||
           (model == PI_MODEL_2) ||
           (model == PI_MODEL_3AP) ||
           (model == PI_MODEL_3B) || (model == PI_MODEL_3BP) ||
           (model == PI_MODEL_4B) || (model == PI_MODEL_400) || (model == PI_MODEL_CM4) ||
           (model == PI_MODEL_ZERO) || (model == PI_MODEL_ZERO_W) || (model == PI_MODEL_ZERO_2W))
    piPlusReadall(model);
  else if ((model == PI_MODEL_CM) || (model == PI_MODEL_CM3) || (model == PI_MODEL_CM3P))
    allReadall();
  else if (model == PI_MODEL_NO_RASPBERRY)
    aw_readall();
  else
    printf("Oops - unable to determine board type... model: %d\n", model);
}

/*
 * doAllReadall:
 *	Force reading of all pins regardless of Pi model
 *********************************************************************************
 */

void doAllReadall(void)
{
  allReadall();
}

/*
 * doQmode:
 *	Query mode on a pin
 *********************************************************************************
 */

void doQmode(int argc, char *argv[])
{
  int pin;

  if (argc != 3)
  {
    fprintf(stderr, "Usage: %s qmode pin\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  pin = atoi(argv[2]);
  printf("%s\n", Board_get_altsName()[getAlt(pin)]);
}
