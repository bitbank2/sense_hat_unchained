//
// Sense Hat Unchained Demo Program
// Initializes the sensors and displays various values
// until the joystick is pressed
//
// Copyright (c) 2017 BitBank Software, Inc.
// written by Larry Bank
//

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sensehat.h>
//
// Update a simple pattern on the LED matrix
//
void UpdatePattern(void)
{
static int iOffset = 0;
int i, j, x, y;
uint16_t color;

	for (i=0; i<64; i++) // do 64 pixels (whole matrix)
	{
		j = i + iOffset;
		color = ((j & 8)>>3) * (j & 31); // blue
		color |= (((j & 16)>>4) * (j & 63)<<5); // green
		color |= (((j & 32)>>5) * (j & 31)<<11); // red
		x = i & 7;
		y = (i >> 3);
		shSetPixel(x, y, color, 0);
	}
	shSetPixel(x, y, color, 1); // force an update
	iOffset++; // increment for next time through
} /* UpdatePattern() */

int main(int argc, char *argv[])
{
int i;
unsigned char ucKeys;

	if (shInit(0) == 0) // Open I2C	
	{
		printf("Unable to open sense hat; is it connected?\n");
		return -1;
	}
	ucKeys = 0;
	while (ucKeys == 0) // run until joystick pressed
	{
	int t, h, p, x, y, z;
		ucKeys = shReadJoystick();
		printf("keys = %02x\n", ucKeys);
		if (shGetTempHumid(&t, &h))
		{
			printf("T=%d, H=%d\n", t, h);
		}
		if (shGetPressure(&p, &t))
		{
			printf("P=%d, T=%d\n", p, t);
		}
		if (shGetMagneto(&x, &y, &z))
		{
			printf("accX=%d, accY=%d, accZ=%d\n", x, y, z);
		}
		for (i=0; i<5; i++) // update pixels 5x as fast as sensor readings
		{
			UpdatePattern();
			usleep(100000);
		}
	} // while waiting for joystick press
	shShutdown();
	return 0;
} /* main() */
