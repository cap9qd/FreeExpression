/*
 * The display is composed of an UG-2864ASYDT01 OLED module using an SSD1325 display driver chip.
 * The graphics library used is u8glib
 *
 * https://code.google.com/p/u8glib/
 *
 * Device type can be either u8g_dev_ssd1325_nhd27oled_bw_sw_spi or the 2x variant, 
 * u8g_dev_ssd1325_nhd27oled_2x_bw_sw_spi
 *
 * https://code.google.com/p/u8glib/wiki/device
 *
 * The UG-2864ASYDT01 module uses mainly four pins to interface with the AVR:
 *
 *  SCK: PORTF, Bit 6
 *  MOSI: PORTF, Bit 5
 *  CS: PORTE, Bit 7
 *  A0: PORTF, Bit 7
 *
 * On the motherboard of the Cricut Expression, the 10 pin header has the following
 * connections:
 *
 * Pin 	|  Display	| AVR
 *------+-------+---------
 *  1   |  GND 	   | NC
 *  2   |  Vcc     | NC
 *  3   |  D1/SDIN | PF5
 *  4   |  D0/SCLK | PF6
 *  5   |  DC#     | PF7
 *  6   |  NC	   | PE4
 *  7   |  NC	   | PE5
 *  8   |  RES	   | PE6
 *  9   |  CS#	   | PE7
 *  10  |  NC      | 
 *
 * This file is part of FreeExpression.
 *
 * https://github.com/thetazzbot/FreeExpression
 *
 * FreeExpression is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 *
 * FreeExpression is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FreeExpression. If not, see http://www.gnu.org/licenses/.
 *
 */

/*
 TODO:  Replace the u8glib, it is adding so much lag to the UI.  I need to hand-craft
 a faster interface.
 */
#ifdef MACHINE_EXPRESSION

#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "display_oled.h"
#include "timer.h"

void _oled_display_speed(void);
void _oled_display_pressure(void);

#ifndef USE_U8G2
#include "./m2u8/u8g.h"

u8g_t u8g;

static uint8_t cur_x=0,cur_y=0;
char display_message[80]; // 4 lines of 20 chars??

// local functions
/*
Displays the currently selected speed as bars
Must be called from inside the draw loop
*/
void _oled_display_speed(void)
{
	int p=timer_get_stepper_speed();
	// at least one bar will always show
	u8g_SetColorIndex(&u8g, 3);
	u8g_DrawBox(&u8g, 30,62,2,2); // 1
	if(p<2) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 33,61,2,3); // 2
	if(p<3) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 36,60,2,4); // 3
	if(p<4) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 39,59,2,5); // 4
	if(p<5)  u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 42,58,2,6); // 5
	u8g_SetColorIndex(&u8g, 3);
}

/*
Displays the currently selected pressure as bars
Must be called from inside the draw loop
*/
void _oled_display_pressure(void)
{
	int p=timer_get_pen_pressure();
	u8g_SetColorIndex(&u8g, 3);
	u8g_DrawBox(&u8g, 54,62,2,2); // 1
	if(p<2) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 57,61,2,3); // 2
	if(p<3) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 60,60,2,4); // 3
	if(p<4) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 63,59,2,5); // 4
	if(p<5) u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 66,58,2,6); // 5
	u8g_SetColorIndex(&u8g, 3);
}


void oled_display_firstpage(void)
{
	u8g_FirstPage(&u8g);
}

int oled_display_nextpage(void)
{
	return u8g_NextPage(&u8g);
}

void oled_display_init(void)
{
	/* select minimal prescaler (max system speed) */
	CLKPR = 0xFF;
	
	                                      // CriCut Pins:      F6        F5        E7        F7        E6 
	                                      // u8g_InitSPI(x, y, SCK,      MOSI,     CS,       A0,       RESET );
                                          //                   SCK       MOSI      CS        D/C
	u8g_InitSPI(&u8g, &u8g_dev_ssd1325_nhd27oled_2x_gr_sw_spi, PN(5, 6), PN(5, 5), PN(4, 7),PN(5,7), PN(4,6));
	u8g_SetRot180(&u8g);
	
	/* assign default color value */
	u8g_SetColorIndex(&u8g, 3);         /* max intensity */
	//  u8g_SetColorIndex(&u8g, 1);         /* pixel on */

	u8g_SetFont(&u8g, u8g_font_profont11);
	u8g_SetFontRefHeightExtendedText(&u8g);
	u8g_SetDefaultForegroundColor(&u8g);
	u8g_SetFontPosTop(&u8g);
}

// main screen redraw functions
void oled_display_update(void)
{
	oled_display_firstpage();
	do {
		u8g_DrawStr(&u8g, cur_x, cur_y, display_message);
		_oled_display_speed();
		_oled_display_pressure();
	} while(oled_display_nextpage());
}

/**
* Displays a string using u8glib's "picture loop".  There is
* significant lag happening here.
*/
void oled_display_puts(const char *s) {
	strcpy(display_message,s);
	oled_display_update();
}



/*
* Horrible code that needs to go away and be replaced by something better
*/

void oled_display_println(char *s) {
	oled_display_puts(s);
	cur_y+=10;
	cur_x=0;
}

#else
//=============================================================================
// For g8u2 attempt...
//----------------------------------------------------------------------------
#include "u8g2.h"
#include <util/delay.h>

u8x8_t u8x8;

static uint8_t cur_x=0,cur_y=0;
char display_message[80]; // 4 lines of 20 chars??

#define DISPLAY_CLK_DIR DDRF
#define DISPLAY_CLK_PORT PORTF
#define DISPLAY_CLK_PIN 6

#define DISPLAY_DATA_DIR DDRF
#define DISPLAY_DATA_PORT PORTF
#define DISPLAY_DATA_PIN 5

#define DISPLAY_CS_DIR DDRE
#define DISPLAY_CS_PORT PORTE
#define DISPLAY_CS_PIN 7

#define DISPLAY_DC_DIR DDRF
#define DISPLAY_DC_PORT PORTF
#define DISPLAY_DC_PIN 7

#define DISPLAY_RESET_DIR DDRE
#define DISPLAY_RESET_PORT PORTE
#define DISPLAY_RESET_PIN 6

#define P_CPU_NS (1000000000UL / F_CPU)

uint8_t u8x8_avr_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	uint8_t cycles;

	switch(msg)
	{
		case U8X8_MSG_DELAY_NANO:     // delay arg_int * 1 nano second
		// At 20Mhz, each cycle is 50ns, the call itself is slower.
		break;
		case U8X8_MSG_DELAY_100NANO:    // delay arg_int * 100 nano seconds
		// Approximate best case values...
		#define CALL_CYCLES 26UL
		#define CALC_CYCLES 4UL
		#define RETURN_CYCLES 4UL
		#define CYCLES_PER_LOOP 4UL

		cycles = (100UL * arg_int) / (P_CPU_NS * CYCLES_PER_LOOP);

		if(cycles > CALL_CYCLES + RETURN_CYCLES + CALC_CYCLES)
		break;

		__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (cycles) : "0" (cycles) // 2 cycles
		);
		break;
		case U8X8_MSG_DELAY_10MICRO:    // delay arg_int * 10 micro seconds
		for(int i=0 ; i < arg_int ; i++)
		_delay_us(10);
		break;
		case U8X8_MSG_DELAY_MILLI:      // delay arg_int * 1 milli second
		for(int i=0 ; i < arg_int ; i++)
		_delay_ms(1);
		break;
		default:
		return 0;
	}
	return 1;
}


uint8_t u8x8_avr_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	// Re-use library for delays

	switch(msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:  // called once during init phase of u8g2/u8x8
		DISPLAY_CLK_DIR |= 1<<DISPLAY_CLK_PIN;
		DISPLAY_DATA_DIR |= 1<<DISPLAY_DATA_PIN;
		DISPLAY_CS_DIR |= 1<<DISPLAY_CS_PIN;
		DISPLAY_DC_DIR |= 1<<DISPLAY_DC_PIN;
		DISPLAY_RESET_DIR |= 1<<DISPLAY_RESET_PIN;
		break;              // can be used to setup pins
		case U8X8_MSG_GPIO_SPI_CLOCK:        // Clock pin: Output level in arg_int
		if(arg_int)
		DISPLAY_CLK_PORT |= (1<<DISPLAY_CLK_PIN);
		else
		DISPLAY_CLK_PORT &= ~(1<<DISPLAY_CLK_PIN);
		break;
		case U8X8_MSG_GPIO_SPI_DATA:        // MOSI pin: Output level in arg_int
		if(arg_int)
		DISPLAY_DATA_PORT |= (1<<DISPLAY_DATA_PIN);
		else
		DISPLAY_DATA_PORT &= ~(1<<DISPLAY_DATA_PIN);
		break;
		case U8X8_MSG_GPIO_CS:        // CS (chip select) pin: Output level in arg_int
		if(arg_int)
		DISPLAY_CS_PORT |= (1<<DISPLAY_CS_PIN);
		else
		DISPLAY_CS_PORT &= ~(1<<DISPLAY_CS_PIN);
		break;
		case U8X8_MSG_GPIO_DC:        // DC (data/cmd, A0, register select) pin: Output level in arg_int
		if(arg_int)
		DISPLAY_DC_PORT |= (1<<DISPLAY_DC_PIN);
		else
		DISPLAY_DC_PORT &= ~(1<<DISPLAY_DC_PIN);
		break;
		
		case U8X8_MSG_GPIO_RESET:     // Reset pin: Output level in arg_int
		if(arg_int)
		DISPLAY_RESET_PORT |= (1<<DISPLAY_RESET_PIN);
		else
		DISPLAY_RESET_PORT &= ~(1<<DISPLAY_RESET_PIN);
		break;
		default:
		if (u8x8_avr_delay(u8x8, msg, arg_int, arg_ptr))	// check for any delay msgs
		return 1;
		u8x8_SetGPIOResult(u8x8, 1);      // default return value
		break;
	}
	return 1;
}

void _oled_display_speed(void)
{
	char tempStr[5];
	int s=timer_get_stepper_speed();
	sprintf(&tempStr[0], "%d", s);
	u8x8_SetFont(&u8x8, u8x8_font_chroma48medium8_r);
	u8x8_DrawString(&u8x8, 4, 6, tempStr);
}

/*
Displays the currently selected pressure as bars
Must be called from inside the draw loop
*/
void _oled_display_pressure(void)
{
	char tempStr[5];
	int p=timer_get_pen_pressure();
	sprintf(&tempStr[0], "%d", p);
	u8x8_SetFont(&u8x8, u8x8_font_chroma48medium8_r);
	u8x8_DrawString(&u8x8, 7, 6, tempStr);
}


void oled_display_firstpage(void)
{
	return;
}

int oled_display_nextpage(void)
{
	return(0);
}

void oled_display_init(void)
{
	CLKPR = 0xFF; //Select minimal pre-scaler (max system speed)
	
	u8x8_Setup(&u8x8, u8x8_d_ssd1325_nhd_128x64, u8x8_cad_001, u8x8_byte_4wire_sw_spi, u8x8_avr_gpio_and_delay);
	u8x8_InitDisplay(&u8x8);
	u8x8_SetFlipMode(&u8x8, 1);
	
	u8x8_SetPowerSave(&u8x8, 0);
	
	u8x8_SetFont(&u8x8, u8x8_font_5x7_f);
	cur_y = cur_x = 0;
	oled_display_puts("FreeExpressionwith G8X8!");
}

// Function to find the minimum of two integers
int min(int a, int b) {
	return (a < b) ? a : b; // Using the ternary operator
}

#define MAX_LINE_LEN 14

// main screen redraw functions
void oled_display_update(void)
{
	u8x8_ClearDisplay(&u8x8);
	_oled_display_speed();
	_oled_display_pressure();
	cur_x = cur_y = 0;
	
	size_t msg_len = strlen(display_message);
	char temp_str[MAX_LINE_LEN+1];

	//Write message total length to bottom row at x=0
	//sprintf(&temp_str[0], "%d", msg_len);
	//u8x8_DrawString(&u8x8, 0, 6, temp_str);
	
	if ( msg_len >= MAX_LINE_LEN ) {
		for(int idx = 0; idx <= min(5, msg_len/MAX_LINE_LEN); idx++){
			msg_len = min(strlen(&display_message[idx*MAX_LINE_LEN]), MAX_LINE_LEN);
			
			memset(&temp_str[0], 0, MAX_LINE_LEN+1); // Fills the entire buffer with zeros
			strncpy(&temp_str[0], &display_message[0]+idx*MAX_LINE_LEN, msg_len);
			u8x8_SetFont(&u8x8, u8x8_font_chroma48medium8_r);
			u8x8_DrawString(&u8x8, 0, idx, temp_str);
			
			// Write message length to bottom row x=12
			//sprintf(&temp_str[0], "%d", msg_len);
			//u8x8_DrawString(&u8x8, 12, 6, temp_str);
		}
	}
	else {
		u8x8_SetFont(&u8x8, u8x8_font_chroma48medium8_r);
		u8x8_DrawString(&u8x8, cur_x, cur_y, display_message);
	}
}

void oled_display_puts(const char *s) 
{
	strcpy(display_message, s);
	oled_display_update();
}

/*
* Horrible code that needs to go away and be replaced by something better
*/

void oled_display_println(char *s) {
	oled_display_puts(s);
	cur_y+=10;
	cur_x=0;
}
//----------------------------------------------------------------------------
//=============================================================================

#endif //#ifndef USE_U8G2
#endif //#ifdef MACHINE_EXPRESSION