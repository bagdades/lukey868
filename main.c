/*
 * main.c
 *
 *  Created on: 11 лют. 2011
 *      Author: vovan
 */

#include "inc.h"
int16_t pwmCounter;
int16_t pwm;
int16_t error;
int16_t integralErr;
int16_t preErr;
int16_t diffErr;
int16_t tempTemp = 0;

int main(void)
{
	Timer0Init();
	/* Timer1Init(); */
	ADCInit();
	Init();
	while(BUT_PIN & (1 << TEST_SOLD))
	{
		//TODO
		ShowError();
		if (flag.lcdUpdate) 
		{
			flag.digPoint = 0;
			LcdUpdate();	
		}
		if (flag.lcdCancel) 
		{
			LcdCancel();	
		}
	}
	//дві секунди показуємо встановлену температуру
	countSetTempVisible = TIME_SET_TEMP_VISIBLE;
	flag.setTempVisible = TRUE;
	/* TCCR1A |= (1 << CS10) | (1 << CS12) #<{(| start timer1 clkio/1024 |)}># */
	while(1)
	{
		if (BUT_PIN & (1 << TEST_SOLD)) 
		{
			//TODO
			/* TCCR1A &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); #<{(| Stop timer1 |)}># */
			/* PORTB |= (1 << PB1);                #<{(| Heater off |)}># */
			ShowError();
			flag.noheater = 1;
			if (flag.lcdUpdate) 
			{
				flag.digPoint = 0;
				LcdUpdate();		
			}
			if (flag.lcdCancel) 
			{
				LcdCancel();	
			}
		}
		else
		{
			if (flag.noheater == 1) 
			{
				/* TCCR1A |= (1 << CS10) | (1 << CS12) #<{(| start timer1 clkio/1024 |)}># */
				flag.noheater = 0;	
			}
			if(flag.keyScan)
			{
				KeyScan();
				if(keyPressed)
				{
					if(keyPressed == _UP)
					{
						workTemp++;
						if(workTemp > MAX_TEMP)
							workTemp = MAX_TEMP;
						countTimeWriteEeprom = TIME_WRITE_EEPROM;
						flag.tempEepromWrite = TRUE;
						countSetTempVisible = TIME_SET_TEMP_VISIBLE;
						flag.setTempVisible = TRUE;
					}
					else if(keyPressed == _DOWN)
					{
						workTemp --;
						if(workTemp < MIN_TEMP)
							workTemp = MIN_TEMP;
						countTimeWriteEeprom = TIME_WRITE_EEPROM;
						flag.tempEepromWrite = TRUE;
						countSetTempVisible = TIME_SET_TEMP_VISIBLE;
						flag.setTempVisible = TRUE;
					}
				}
			}
			if(flag.adcRead)
			{
				/*
				 * Період ШІМ 100мс
				 * Зчитування adc 10мс
				 * на одиницю регулюючого сигналу припадає 1 півперіод (2%)
				 */
				int16_t tempAdc;
				adcResult = ADCRead(0);
				tempAdc = (workTemp * 14) / 10;//переведення встановленої температури
				//відповідно до adc
				pwmCounter ++;
				tempTemp += adcResult;
				if(pwmCounter >= 40)
				{
					pwmCounter = 0;
					temperatura = tempTemp / 56;
					tempTemp = 0;
				}
				error = tempAdc - adcResult;
				diffErr = error - preErr;
				/* pwm = 2 * error + integralErr + 2 * diffErr; */
				pwm = error + integralErr / 10 + 2 * diffErr;
				/* if(pwm >=0 && pwm <= 20) */
					integralErr += error;
				/* if(integralErr > 10) */
				/* 	integralErr = 10; */
				preErr = error;//поточна помилка стає попередньою (використовується для розрахунку
				//диференціальної складової)
				/*
				 * якщо коротке замикання або обрив датчика ,
				 * відключаємо підігрів
				 */
				/* if(pwm < pwmCounter || adcResult > 1000 || adcResult < 3) */
				if(pwm < pwmCounter || adcResult > 1000)
				{
					CONTR_PORT |= (1 << CONTR_OUT);
					flag.digPoint = 0;
				}
				else {
					CONTR_PORT &= ~(1 << CONTR_OUT);
					flag.digPoint = 1;// індикація нагріву
				}
			}
			if(flag.setTempVisible)
			{
				ResultBcd(workTemp);
			}
			else 
			{
				ResultBcd(temperatura);
			}
			//обновлення індикації кожних 5мс
			if(flag.lcdUpdate)
			{
				LcdUpdate();
			}
			if (flag.lcdCancel) 
			{
				LcdCancel();	
			}
			//зчитування температури кожної пів секунди
			if(flag.readTemp)
			{
				flag.readTemp = FALSE;
				/* temperatura = (adcResult * 10) / 14; */
			}
			if(flag.eepromWrite)
			{
				SaveEepromMode();
			}
		}
	}
}
