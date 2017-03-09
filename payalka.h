/*
 * payalka.h
 *
 *  Created on: 11 лют. 2011
 *      Author: vovan
 */

#ifndef PAYALKA_H_
#define PAYALKA_H_

#include <inttypes.h>
#include <avr/pgmspace.h>

//#define DEBUG

//LCD
#define LCD_PORT 	PORTD
#define LCD_DDR		DDRD
#define DIG_PORT	PORTB
#define DIG_DDR		DDRB
#define DIG_1		0
#define DIG_2		7
#define DIG_3		6
/* #define DIG_4		2 */
#define DIG_P		4
#define DIG_MASK	((1 << DIG_1)|(1 << DIG_2)|(1 << DIG_3))

//BUTTON
#define BUT_PORT 	PORTB
#define BUT_DDR		DDRB
#define BUT_PIN		PINB
#define UP			5
#define DOWN		2
#define TEST_SOLD	4
#define BUT_MASK	((1 << UP)|(1 << DOWN)|(1 << TEST_SOLD))
#define DELAY_SHORT	1
#define DELAY_LONG	30

//TIMER
#define T0_PRESC	1024UL
#define T0_FREQ		1000UL
#define T0_INIT		255 - (F_CPU/(T0_FREQ * T0_PRESC))

//TIME
#define PERIOD_KEY_SCAN			15
#define PERIOD_LCD_UPDATE		7
#define PERIOD_LCD_CANCEL		3
#define PERIOD_ADC_READ			5
#define PERIOD_TEMP_UPDATE		500
#define TIME_WRITE_EEPROM		3000
#define TIME_SET_TEMP_VISIBLE 	2000
#define MAX_TEMP				400
#define MIN_TEMP				100

//CONTROL
#define CONTR_PORT	PORTB
#define CONTR_DDR	DDRB
#define CONTR_OUT	1

#define ADC_VREF_TYPE ((1<<REFS1)|(1<<REFS0))

static const uint8_t sevenSegmentCod[] PROGMEM ={
		0x50,//0
		0x5F,//1
		0x38,//2
		0x1A,//3
		0x17,//4
		0x92,//5
		0x90,//6
		0x5E,//7
		0x10,//8
		0x12,//9
		0xBF //-
};
volatile struct _flag{
	uint16_t keyScan:		1;
	uint16_t lcdUpdate:		1;
	uint16_t lcdCancel:		1;
	uint16_t adcRead:		1;
	uint16_t eepromWrite:	1;
	uint16_t modeEepromWrite:1;
	uint16_t tempEepromWrite:1;
	uint16_t setTempVisible:	1;
	uint16_t digPoint:		1;
	uint16_t readTemp:		1;
	uint16_t noheater:		1;
}flag;

//ENUMERATION
enum keyPad{
	_UP = 1,
	_DOWN,
	_SET
};
enum {
	FALSE = 0,
	TRUE
};
enum {
	MODE_ONE = 0,
	MODE_TWO,
	MODE_THREE,
	MODE_FOUR
};

//EXTERNS
extern uint8_t keyPressed;
extern int16_t workTemp;
extern uint16_t workTempEeprom;
extern volatile uint8_t countTimeKeyScan;
extern volatile uint8_t countTimeLcdUpdate;
extern volatile uint8_t countTimeAdcRead;
extern volatile uint16_t countTimeWriteEeprom;
extern volatile uint16_t countSetTempVisible;
extern uint16_t temperatura;
extern int16_t adcResult;
extern uint8_t dataOut[3];

//PROTOTYPES
void Init			(void);
void Timer0Init		(void);
/* void Timer1Init		(void); */
void KeyScan		(void);
void LcdUpdate		(void);
void LcdCancel		(void);
void ADCInit		(void);
int16_t ADCRead		(uint8_t chanel);
void ResultBcd		(uint16_t data);
void SaveEepromMode	(void);
void ShowError		(void);

#endif /* PAYALKA_H_ */
