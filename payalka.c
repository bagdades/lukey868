/*
 * payalka.c
 *
 *  Created on: 11 лют. 2011
 *      Author: vovan
 */
#include "inc.h"

uint8_t keyPressed;
uint8_t dataOut[3];
/* Масив значень температур для рижимів workTemp */
int16_t workTemp;
/* Режим температур */
uint16_t workTempEeprom EEMEM = 230;
uint16_t temperatura;
int16_t adcResult;

volatile uint8_t countTimeKeyScan = PERIOD_KEY_SCAN;
volatile uint8_t countTimeLcdUpdate = PERIOD_LCD_UPDATE;
volatile uint8_t countTimeLcdCancel = PERIOD_LCD_CANCEL;
volatile uint8_t countTimeAdcRead = PERIOD_ADC_READ;
volatile uint16_t countTimeTempRead = PERIOD_TEMP_UPDATE;
volatile uint16_t countTimeWriteEeprom = TIME_WRITE_EEPROM;
volatile uint16_t countSetTempVisible = TIME_SET_TEMP_VISIBLE;

void Init(void) 
{
	/* uint8_t i; */
	BUT_DDR &= ~BUT_MASK; //port in
	BUT_PORT |= BUT_MASK; //pull-up
	LCD_DDR = 0xFF; //port out
	DIG_DDR |= DIG_MASK; //port out
	DIG_PORT &= ~DIG_MASK; //low level
	CONTR_DDR |= (1 << CONTR_OUT);
	CONTR_PORT |= (1 << CONTR_OUT); /* heater off */
	sei();
	workTemp = eeprom_read_word(&workTempEeprom); //read from eeprom temp value
}

void Timer0Init(void) 
{
#if(F_CPU != 8000000)
#error ***You must set TCCR0
#endif
	TCCR0 |= (1 << CS02) | (1 << CS00); //T0_PRESC = 1024
	TCNT0 = T0_INIT;
	TIMSK |= (1 << TOIE0); //enable interrupt overllow timer0
}

/* void Timer1Init(void) */
/* { */
/* 	DDRB |= 1 << PB1;       #<{(| OC1A output |)}># */
/* 	PORTB |= 1 << PB1;      #<{(| Of heater |)}># */
/* 	TCCR1A |= (1 << WGM10); */
/* 	TCCR1B |= (1 << WGM12); #<{(| Fast PWM 8 bit |)}># */
/* 	TCCR1A |= (1 << COM1A1) | (1 << COM1A0); #<{(| Set 1 if tcnt = ocr, set 0 if tcnt = 0 |)}># */
/* } */

ISR(TIMER0_OVF_vect) 
{
	TCNT0 = T0_INIT;
	countTimeKeyScan--;
	countTimeLcdUpdate--;
	countTimeAdcRead--;
	countTimeTempRead--;
	if (!countTimeKeyScan) 
	{
		countTimeKeyScan = PERIOD_KEY_SCAN;
		flag.keyScan = TRUE;
	}
	if (!countTimeLcdUpdate) 
	{
		countTimeLcdUpdate = PERIOD_LCD_UPDATE;
		flag.lcdUpdate = TRUE;
	}
	if (!countTimeAdcRead) 
	{
		countTimeAdcRead = PERIOD_ADC_READ;
		flag.adcRead = TRUE;
	}
	if (!countTimeTempRead) 
	{
		countTimeTempRead = PERIOD_TEMP_UPDATE;
		flag.readTemp = TRUE;
	}
	if (countTimeLcdCancel) 
	{
		countTimeLcdCancel--;
		if(countTimeLcdCancel == 0)
			flag.lcdCancel = TRUE;
	}
	if (countTimeWriteEeprom) 
	{
		countTimeWriteEeprom--;
		if (countTimeWriteEeprom == 0)
			flag.eepromWrite = TRUE;
	}
	if (countSetTempVisible) 
	{
		countSetTempVisible--;
		if (countSetTempVisible == 0)
			flag.setTempVisible = FALSE;
	}
}

void KeyScan(void) 
{
	static uint8_t temp = 0;
	uint8_t key;
	keyPressed = 0;
	if (bit_is_clear(BUT_PIN,UP))
		key = _UP;
	else if (bit_is_clear(BUT_PIN,DOWN))
		key = _DOWN;
	/* else if (bit_is_clear(BUT_PIN,SET)) */
	/* 	key = _SET; */
	else
		key = 0;
	if (key) {
		if (temp >= DELAY_LONG) 
		{
			temp -= 1;
			keyPressed = key;
			flag.keyScan = FALSE;
			return;
		}
		if (temp == DELAY_SHORT)
			keyPressed = key;
		temp++;
		flag.keyScan = FALSE;
		return;
	} else
		temp = 0;
	flag.keyScan = FALSE;
}

void LcdUpdate(void) 
{
	static uint8_t count = 0;
	DIG_PORT &= ~DIG_MASK;
	if (count == 0) 
	{
		DIG_PORT |= (1 << DIG_3);
		LCD_PORT = pgm_read_byte(sevenSegmentCod + dataOut[count]);
	} else 
	if (count == 1) 
	{
		DIG_PORT |= (1 << DIG_2);
		LCD_PORT = pgm_read_byte(sevenSegmentCod + dataOut[count]);
	} else 
	if (count == 2) 
	{
		DIG_PORT |= (1 << DIG_1);
		LCD_PORT = pgm_read_byte(sevenSegmentCod + dataOut[count]);
		if (flag.digPoint)
			LCD_PORT &= ~(1 << DIG_P);
		else LCD_PORT |= (1 << DIG_P);
	}
	count++;
	if (count > 2)
		count = 0;
	flag.lcdUpdate = FALSE;
	countTimeLcdCancel = PERIOD_LCD_CANCEL;
}

void LcdCancel(void)
{
	DIG_PORT &= ~DIG_MASK;
	flag.lcdCancel = FALSE;
}

void ADCInit(void) 
{
#if(F_CPU != 8000000)
#error ***You must set ADCSRA
#endif
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADEN); //PRESC F_CPU/64
	ADMUX &= ~ADC_VREF_TYPE; //vref
}

int16_t ADCRead(uint8_t chanel) 
{
	ADMUX &= ~ADC_VREF_TYPE; //vref
	ADMUX &= 0xF8;
	ADMUX |= chanel;
	_delay_us(10); //delay needed for stabilization of the ACD input voltag
	ADCSRA |= (1 << ADSC); //start convertion
	while ((ADCSRA & (1 << ADIF)) == 0)
		; //wait for complete conversion
	ADCSRA |= (1 << ADIF);
	flag.adcRead = FALSE;
	return ADCW;
}

void ResultBcd(uint16_t data) 
{
	/* uint8_t i; */
	/* if (adcResult > 1000 || adcResult < 3)  */
	/* { //якщо обрив датчика ,або коротке замикання */
	/* 	for (i = 1; i < 3; ++i)  */
	/* 	{ */
	/* 		dataOut[i] = 10; //виводимо на дисплей рисочки */
	/* 	} */
	/* 	return; */
	/* } */
	dataOut[0] = data / 100;
	dataOut[1] = (data / 10) % 10;
	dataOut[2] = data % 10;
}

void SaveEepromMode(void) 
{
	if (flag.tempEepromWrite) 
	{
		eeprom_write_word(&workTempEeprom, workTemp);
		flag.tempEepromWrite = FALSE;
	}
	flag.eepromWrite = FALSE;
}

void ShowError(void)
{
	dataOut[0] = 10;
	dataOut[1] = 10;
	dataOut[2] = 10;
}
