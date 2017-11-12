#ifndef _SENSEHAT_H_
#define _SENSEHAT_H_
//
// Sense Hat Unchained - function definitions
//
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// bit definitions for joystick
#define JOY_DOWN 1
#define JOY_UP   4
#define JOY_LEFT 16
#define JOY_RIGHT 2
#define JOY_ENTER 8

//
// Returns the air pressure in hPa
// 1 = pressure successfully read
// 0 = failed to read pressure
//
int shGetPressure(int *pPressure, int *pTemp);

//
// Read the 5-way joystick status
// 
unsigned char shReadJoystick(void);

//
// Read the magnetometer values for x/y/z
//
int shGetMagneto(int *Mx, int *My, int *Mz);

//
// Set the pixel the given color. The display will flicker
// if it's updated for each pixel change, so it's best
// to change the pixels for the current "frame", then
// enable the update to the array. The color is in RGB565
// format and the valid range of x/y is 0-7
//
int shSetPixel(int x, int y, uint16_t color, int bUpdate);

//
// Initialize the sensors on the Sense Hat
// iChannel = I2C channel number (0/1/2)
//
int shInit(int iChannel);

//
// Returns the temperature and humidity
//
int shGetTempHumid(int *Temp, int *Humid);

//
// Frees resources and closes handles
//
void shShutdown(void);

#endif // _SENSEHAT_H_
