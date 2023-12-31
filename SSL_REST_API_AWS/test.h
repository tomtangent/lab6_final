/*
 * test.h
 *
 *  Created on: Apr 16, 2023
 *      Author: Tom
 */

#ifndef TEST_H_
#define TEST_H_

extern int cursor_x;
extern int cursor_y;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//void drawChar(int x, int y, unsigned char c,
//unsigned int color, unsigned int bg, unsigned char size)

void printChar();

void printDigit(char c);

void testHelloWorld();

//*****************************************************************************
//  function delays 3*ulCount cycles
void delay(unsigned long ulCount);

//*****************************************************************************
void testfastlines(unsigned int color1, unsigned int color2);

//*****************************************************************************

void testdrawrects(unsigned int color);

//*****************************************************************************

void testfillrects(unsigned int color1, unsigned int color2);

//*****************************************************************************

void testfillcircles(unsigned char radius, unsigned int color);

//*****************************************************************************

void testdrawcircles(unsigned char radius, unsigned int color);

//*****************************************************************************

void testtriangles();
//*****************************************************************************

void testroundrects();

//*****************************************************************************
void testlines(unsigned int color);

//*****************************************************************************

void lcdTestPattern(void);
/**************************************************************************/
void lcdTestPattern2(void);

/**************************************************************************/





#endif /* TEST_H_ */
