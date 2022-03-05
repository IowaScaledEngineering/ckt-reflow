/*************************************************************************
:    Reflow Oven Controller
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2022 Nathan D. Holmes (maverick@drgw.net)
     & Michael Petersen (railfan@drgw.net)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "lcd.h"
#include "avr-i2c-master.h"


#define SOAK_TEMP     200
#define SOAK_TIME     900
#define REFLOW_TEMP   230
#define REFLOW_TIME   900



volatile uint16_t decisecs=0;
volatile uint16_t countdownDecisecs=0;

volatile uint8_t events=0;

#define EVENT_READ_INPUTS   0x01
#define EVENT_UPDATE_SCREEN 0x02
#define EVENT_SECOND_TICK   0x04
#define EVENT_SEND_DMX      0x80


#define SOFTKEY_1      0x0001
#define SOFTKEY_2      0x0002
#define SOFTKEY_3      0x0004
#define SOFTKEY_4      0x0008
#define SOFTKEY_1_LONG 0x0010
#define SOFTKEY_2_LONG 0x0020
#define SOFTKEY_3_LONG 0x0040
#define SOFTKEY_4_LONG 0x0080


#define BIGCHAR_DEG_C    10
#define BIGCHAR_SPACE    13


#define MCP9600_I2C_ADDR    0x60


typedef enum
{
	IDLE_DRAW,
	IDLE,
	MANUAL_DRAW,
	MANUAL,
	RAMP1_DRAW,
	RAMP1,
	SOAK_DRAW,
	SOAK,
	RAMP2_DRAW,
	RAMP2,
	REFLOW_DRAW,
	REFLOW,
	DONE_DRAW,
	DONE,
} ReflowState;

void initialize100HzTimer(void)
{
	// Set up timer 3 for 100Hz interrupts
	TCNT3 = 0;
	OCR3A = 0x0752;
	TCCR3A = 0;
	TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
	TCCR3C = 0;
	TIFR3 |= _BV(OCF3A);
	TIMSK3 |= _BV(OCIE3A);
	
	decisecs = 0;
}

ISR(TIMER3_COMPA_vect)
{
	static uint8_t ticks = 0;
	static uint8_t screenUpdateTicks = 0;
	
	if (ticks & 0x01)
		events |= EVENT_READ_INPUTS;

	if (++screenUpdateTicks >= 100)
	{
		events |= EVENT_UPDATE_SCREEN  | EVENT_SECOND_TICK;
		screenUpdateTicks = 0;
	}

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
		if(countdownDecisecs)
			countdownDecisecs--;
	}
}

#define LCD_BACKLIGHT_PIN  PB4

void lcd_backlightOn()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB |= _BV(LCD_BACKLIGHT_PIN);
}

void lcd_backlightOff()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB &= ~(_BV(LCD_BACKLIGHT_PIN));
}

typedef struct
{
	uint16_t clock_A;
	uint16_t clock_B;
	uint16_t debounced_state;
} DebounceState;

void initDebounceState(DebounceState* d, uint16_t initialState)
{
	d->clock_A = d->clock_B = 0;
	d->debounced_state = initialState;
}

uint16_t debounce(uint16_t raw_inputs, DebounceState* d)
{
	uint16_t delta = raw_inputs ^ d->debounced_state;   //Find all of the changes
	uint16_t changes;

	d->clock_A ^= d->clock_B;                     //Increment the counters
	d->clock_B  = ~d->clock_B;

	d->clock_A &= delta;                       //Reset the counters if no changes
	d->clock_B &= delta;                       //were detected.

	changes = ~((~delta) | d->clock_A | d->clock_B);
	d->debounced_state ^= changes;
	return(changes & ~(d->debounced_state));
}

#define PANEL_SWITCH_MASK (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

void initializeSwitches(void)
{
	DDRC &= ~(PANEL_SWITCH_MASK);  // Make inputs
	PORTC |= (PANEL_SWITCH_MASK);  // Turn on pull-ups
}

uint16_t readSwitches()
{
	uint16_t switchStates = (PINC & PANEL_SWITCH_MASK)>>2;
	return switchStates;
}

void mcp9600_init(void)
{
	uint8_t i2cBuf[8];
	i2cBuf[0] = (MCP9600_I2C_ADDR << 1);
	i2cBuf[1] = 0x05;  // Sensor Configuration
	i2cBuf[2] = 0x00;  // Type K, no filter
	i2c_transmit(i2cBuf, 3, 1);
	
	i2cBuf[0] = (MCP9600_I2C_ADDR << 1);
	i2cBuf[1] = 0x06;  // Device Configuration
	i2cBuf[2] = 0x00;
	i2c_transmit(i2cBuf, 3, 1);
}

uint16_t mcp9600_readTemperature(void)
{
	uint8_t successful = 0;
	uint8_t i2cBuf[8];
	i2cBuf[0] = (MCP9600_I2C_ADDR << 1);
	i2cBuf[1] = 0x00;  // TH register
	i2c_transmit(i2cBuf, 2, 1);

	_delay_us(300);  // Helps prevent read errors

	i2cBuf[0] = (MCP9600_I2C_ADDR << 1) | 0x01;
	i2c_transmit(i2cBuf, 3, 1);

	while(i2c_busy());

	successful = i2c_receive(i2cBuf, 3);
	
	if(!successful)
		return 0;
	
	// Return in deg C (drop the lower 4 fractional bits)
	return ((i2cBuf[1] << 4) + (i2cBuf[2] >> 4));
}


#define BUZZER_PIN_A  PD4
#define BUZZER_PIN_B  PD5

void disableBuzzer(void)
{
	PORTD &= ~_BV(BUZZER_PIN_A);
	PORTD &= ~_BV(BUZZER_PIN_B);
}

void enableBuzzer(void)
{
	PORTD &= ~_BV(BUZZER_PIN_A);
	PORTD |= _BV(BUZZER_PIN_B);
}


#define OVEN_PIN  PD6

void disableOven(void)
{
	PORTD &= ~_BV(OVEN_PIN);
	lcd_gotoxy(17,0);
	lcd_puts("OFF");
}

void enableOven(void)
{
	PORTD |= _BV(OVEN_PIN);
	lcd_gotoxy(17,0);
	lcd_puts(" ON");
}


void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_1S);

	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	DDRD |= _BV(OVEN_PIN);
	disableOven();

	DDRD |= _BV(BUZZER_PIN_A) | _BV(BUZZER_PIN_B);	
	disableBuzzer();

	initializeSwitches();
	i2c_master_init();
	
	// Set Up LCD Panel
	wdt_reset();
	lcd_backlightOn();
	lcd_init(LCD_DISP_ON);
	lcd_setup_bigdigits();
	lcd_clrscr();
	lcd_gotoxy(0,4);
	lcd_puts("Starting");
	_delay_ms(500);
	lcd_clrscr();
	lcd_gotoxy(0,0);

	// Set up tick timer
	initialize100HzTimer();

	// Enable interrupts
	sei();

	mcp9600_init();
}



void drawSoftKeys_p(const char* key1Text, const char* key2Text, const char* key3Text, const char* key4Text)
{
	uint8_t i;

	lcd_gotoxy(0,3);
	for(i=0; i<20; i++)
		lcd_putc(' ');

	lcd_gotoxy(0,3);
	lcd_puts_p(key1Text);
	lcd_gotoxy(5,3);
	lcd_puts_p(key2Text);
	lcd_gotoxy(10,3);
	lcd_puts_p(key3Text);
	lcd_gotoxy(15,3);
	lcd_puts_p(key4Text);
}


int main(void)
{
	uint8_t buttonsPressed=0;
	uint8_t buttonLongPressCounters[4] = {3,3,3,3};
	DebounceState d;
	ReflowState reflowState = IDLE_DRAW;
	
	uint16_t temperatureReadErrors = 0;
	
	uint16_t celsiusTemperature = 0;
	uint16_t celsiusTemperature_max = 0;
	
	uint16_t counterTemp;
	
	// Application initialization
	init();
	initDebounceState(&d, 0xFFFF);

	wdt_reset();
	
	while (1)
	{
		wdt_reset();

		lcd_backlightOn();
		
		if (events & EVENT_READ_INPUTS)
		{
			events &= ~(EVENT_READ_INPUTS);
			buttonsPressed = debounce(readSwitches(), &d);

			for(uint8_t btn=0; btn<4; btn++)
			{
				if (buttonsPressed & (1<<btn))
					buttonLongPressCounters[btn] = 25; // On initial press, we set a 0.5s delay before rapid

				if (d.debounced_state & (1<<btn))
					buttonLongPressCounters[btn] = 25; // Long delay if the button is up, too
				else
				{
					// Button is down
					if (buttonLongPressCounters[btn])
						buttonLongPressCounters[btn]--;
					else
					{
						buttonsPressed |= (1<<(btn+4));
						buttonLongPressCounters[btn] = 5; // Repeat time
					}
				}
			}
		}

		celsiusTemperature = mcp9600_readTemperature();
		
		if(0 == celsiusTemperature)
			temperatureReadErrors++;

		if(celsiusTemperature > celsiusTemperature_max)
			celsiusTemperature_max = celsiusTemperature;

		// Draw the temperature
		if(celsiusTemperature >= 100)
			lcd_putc_big(0, (celsiusTemperature/100) % 10);
		else
			lcd_putc_big(0, BIGCHAR_SPACE);

		if(celsiusTemperature >= 10)
			lcd_putc_big(1, (celsiusTemperature/10) % 10);
		else
			lcd_putc_big(0, BIGCHAR_SPACE);

		lcd_putc_big(2, (celsiusTemperature) % 10);

		lcd_putc_big(3, BIGCHAR_DEG_C);


		lcd_gotoxy(0,2);
		lcd_puts("Pk:");
		printDec3Dig(celsiusTemperature_max);
		lcd_putc('C');

		lcd_gotoxy(16,1);
		printDec4Dig(temperatureReadErrors);

		switch(reflowState)
		{
			case IDLE_DRAW:
				drawSoftKeys_p(PSTR("START"), PSTR(""), PSTR("ON"), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_puts(" IDLE ");
				disableOven();
				reflowState = IDLE;
				break;
			case IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					celsiusTemperature_max = celsiusTemperature;
					reflowState = RAMP1_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					reflowState = MANUAL_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case MANUAL_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR(""), PSTR("OFF"), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_puts("MANUAL");
				enableOven();
				reflowState = MANUAL;
				break;
			case MANUAL:
				if (SOFTKEY_3 & buttonsPressed)
				{
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case RAMP1_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR("CANCEL"), PSTR(""), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_putc(' ');
				lcd_putc(0x7E);
				printDec3Dig(SOAK_TEMP);
				lcd_putc('C');
				enableOven();
				reflowState = RAMP1;
				break;
			case RAMP1:
				if(celsiusTemperature >= SOAK_TEMP)
				{
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						countdownDecisecs = SOAK_TIME;
					}
					reflowState = SOAK_DRAW;
				}
				if (SOFTKEY_2 & buttonsPressed)
				{
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SOAK_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR("CANCEL"), PSTR(""), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_puts(" SOAK ");
				disableOven();
				reflowState = SOAK;
				break;
			case SOAK:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					counterTemp = countdownDecisecs;
				}
				if(!counterTemp)
				{
					reflowState = RAMP2_DRAW;
				}
				if (SOFTKEY_2 & buttonsPressed)
				{
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case RAMP2_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR("CANCEL"), PSTR(""), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_putc(' ');
				lcd_putc(0x7E);
				printDec3Dig(REFLOW_TEMP);
				lcd_putc('C');
				enableOven();
				reflowState = RAMP2;
				break;
			case RAMP2:
				if(celsiusTemperature >= REFLOW_TEMP)
				{
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						countdownDecisecs = REFLOW_TIME;
					}
					reflowState = REFLOW_DRAW;
				}
				if (SOFTKEY_2 & buttonsPressed)
				{
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case REFLOW_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR("CANCEL"), PSTR(""), PSTR(""));
				lcd_gotoxy(8,2);
				lcd_puts("REFLOW");
				disableOven();
				reflowState = REFLOW;
				break;
			case REFLOW:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					counterTemp = countdownDecisecs;
				}
				if(!counterTemp)
				{
					reflowState = DONE_DRAW;
				}
				if (SOFTKEY_2 & buttonsPressed)
				{
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case DONE_DRAW:
				drawSoftKeys_p(PSTR(""), PSTR(""), PSTR(""), PSTR("RESET"));
				lcd_gotoxy(8,2);
				lcd_puts(" DONE!");
				enableBuzzer();
				reflowState = DONE;
				break;

			case DONE:
				if (SOFTKEY_4 & buttonsPressed)
				{
					disableBuzzer();
					reflowState = IDLE_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;
				break;
		}

		// Print timer
		lcd_gotoxy(16,2);
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			counterTemp = (countdownDecisecs + 5) / 10;
		}
		lcd_putc('0' + counterTemp / 60);
		lcd_putc(':');
		counterTemp = counterTemp % 60;  // extract seconds
		lcd_putc('0' + counterTemp / 10);
		lcd_putc('0' + counterTemp % 10);
	}
}


