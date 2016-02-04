/*
 * LED_thermo_cpp.cpp
 *
 * Created: 30.01.2016 14:26:59
 *  Author: Nik
 */ 
#undef __AVR_ATtiny2313__
#define __AVR_AT90S2313__

#define F_CPU 6000000L
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

	
#define PortReg(port) (*(port))
#define DirReg(port)  (*((port) - 1))
#define PinReg(port)  (*((port) - 2))

#define TIME_TO_DISP_SENSOR 1000 //время отображения температуры от сенсора в мс

#define INIT_TIMER() TCCR0 =  _BV(CS02);\
					 TIMSK |= _BV(TOIE0);\
					 TCNT0 = 0x00;
#define STOP_TIMER() TCCR0 = 0;\
				     TIMSK |= _BV(TOIE0);

#define SEGMENT_PORT (PORTB)
#define SEGMENT_DDR  DirReg(&SEGMENT_PORT)
#define SEGMENT_PIN  PinReg(&SEGMENT_PORT)

#define INIT_SEGMENT_PORT()  (SEGMENT_DDR = 0xFF);\
							 (SEGMENT_PORT = 0xFF);
#define DEINIT_SEGMENT_PORT()  (SEGMENT_DDR = 0x00);\
							   (SEGMENT_PORT = 0x00);
							
#define DISPLAY_SEGMENT(digit,dot) (SEGMENT_PORT = (( ((digit) << 1) & 0xFE) | ((dot) & 0x01)) );

#define DIGIT_SELECT_PORT PortReg(&PORTD)
#define DIGIT_SELECT_DDR  DirReg(&DIGIT_SELECT_PORT)
#define DIGIT_SELECT_PIN  PinReg(&DIGIT_SELECT_PORT)

#define DIGIT_FIRST_PIN		PD6
#define DIGIT_SECOND_PIN	PD5
#define DIGIT_THIRD_PIN		PD4

#define DIGIT_PORT_MASK ( _BV(DIGIT_FIRST_PIN) | _BV(DIGIT_SECOND_PIN) | _BV(DIGIT_THIRD_PIN) )
#define INIT_DIGIT_SELECT_PORT()	(DIGIT_SELECT_DDR = DIGIT_SELECT_DDR | (DIGIT_PORT_MASK));\
									(DIGIT_SELECT_PORT &= ~DIGIT_PORT_MASK);
									
#define DEINIT_DIGIT_SELECT_PORT() (DIGIT_SELECT_DDR = DIGIT_SELECT_DDR & ~(DIGIT_PORT_MASK));\
									(DIGIT_SELECT_PORT &= ~DIGIT_PORT_MASK);


#define SELECT_DIGIT(digit) (DIGIT_SELECT_PORT = (DIGIT_SELECT_PORT & ~DIGIT_PORT_MASK) | ~((_BV(digit)) << DIGIT_THIRD_PIN ) );


const PROGMEM unsigned char  sevenSegNum[10] =
	  {
		  0b01111110, //0+
		  0x06, //1+
		  0x6d, //2+
		  0b01001111, //3+
		  0b00010111, //4+
		  0x5b, //5+
		  0b01111011, //6+
		  0b00001110, //7+
		  0x7f, //8+
		  0b01011111  //9
	  };

void DynamicIndication(signed char  number,char dotNumber){
static char step;
SELECT_DIGIT(step);

switch (step)
{
	case 0:{
		DISPLAY_SEGMENT(0,(number & 0x80)>>7);
		}break;
	case 1:{
		DISPLAY_SEGMENT( pgm_read_byte(sevenSegNum + abs(number)/10),(dotNumber & 0x01));
		}break;
	case 2:{
		DISPLAY_SEGMENT(pgm_read_byte(sevenSegNum + abs(number)%10 ),((dotNumber+1) & 0x01));
		}break;
	}

step++;
step = step%3;
}

signed char g_temp = 0,g_selectSensor = 0;
ISR(TIMER0_OVF0_vect){
	DynamicIndication(g_temp,g_selectSensor);
}

#define PERIPHERIAL_ON_PORT PortReg(&PORTD)
#define PERIPHERIAL_ON_DDR  DirReg(&DIGIT_SELECT_PORT)
#define PERIPHERIAL_ON_PIN  PD0

#define INIT_PERIPHERIAL_ON_PORT()	PERIPHERIAL_ON_DDR |= _BV(PERIPHERIAL_ON_PIN);
#define ON_PERIPHERIA()				PERIPHERIAL_ON_PORT |=  _BV(PERIPHERIAL_ON_PIN);
#define OFF_PERIPHERIA()			PERIPHERIAL_ON_PORT &=  ~_BV(PERIPHERIAL_ON_PIN);

#include "ds18x20.h"
#include "onewire.h"

uint8_t sensorId[8];

#include <avr/sleep.h>
#define CYCLES_BEFORE_SLEEP 1
char g_cyclesToSleep = 0;
char g_timerToSleep = 0;

#define ENABLE_INT0() DDRD |=  _BV(PD2);\
					  PORTD |= _BV(PD2);\
					  GIMSK |= _BV(INT0);
#define DISABLE_INT0() GIMSK &= ~_BV(INT0);

ISR(INT0_vect){
		DISABLE_INT0();
	//	wdt_disable();
		INIT_DIGIT_SELECT_PORT();
		INIT_SEGMENT_PORT();
		INIT_TIMER();
		INIT_PERIPHERIAL_ON_PORT();
		ON_PERIPHERIA();
		_delay_ms(100);
		g_cyclesToSleep = CYCLES_BEFORE_SLEEP;
		ow_reset();
		ow_command( DS18X20_CONVERT_T, 0 );
		_delay_ms(780);
}

#define PORT_TO_RADIO PORTD
#define DDR_TO_RADIO  DDRD
#define PIN_TO_RADIO  PD1

#define INIT_RADIO_PORT() DDR_TO_RADIO |= _BV(PIN_TO_RADIO);\
					      PORT_TO_RADIO &= ~_BV(PIN_TO_RADIO);
#define DEINIT_RADIO_PORT() PORT_TO_RADIO &= ~_BV(PIN_TO_RADIO);//\
							  //DDR_TO_RADIO &= ~_BV(PIN_TO_RADIO);
#define RADIO_PIN_LOW()  PORT_TO_RADIO &= ~_BV(PIN_TO_RADIO);
#define RADIO_PIN_HIGH() PORT_TO_RADIO |= _BV(PIN_TO_RADIO);

void SendRadioBit(char bit){
	RADIO_PIN_LOW();
	if (bit & 0x01)
	{
		RADIO_PIN_LOW();
		_delay_us(100);
		RADIO_PIN_HIGH();
		_delay_us(300);
	}else
	{
		RADIO_PIN_LOW();
		_delay_us(300);
		RADIO_PIN_HIGH();
		_delay_us(100);
	}
}

void SendRadioByte(char data){
	for (char i = 0; i < 8 ; i++)
	{
		SendRadioBit(data);
		data = data>>1;
	}
}
void SendTempDataToBase(){
	RADIO_PIN_HIGH();
	_delay_us(500);
	RADIO_PIN_LOW();
	SendRadioByte(0xBA);
	SendRadioByte(0xBA);
	SendRadioByte(g_selectSensor);
	SendRadioByte(g_temp);
	RADIO_PIN_HIGH();
	_delay_us(150);
	RADIO_PIN_LOW();
	
	DEINIT_RADIO_PORT();
}

#define START_ON_TIMER() TCCR1A = 0;\
						TCCR1B =  (1<<CS12) | (0<<CS11) | (1<<CS10);\
						TIMSK = (1<<TOIE1);
#define STOP_ON_TIMER() TCCR1B = 0;\
						TIMSK = 0;
#define  NUM_OF_11S_CYCLES 1
int g_timeToLeft = 0;

#include <avr/wdt.h>
ISR(TIMER1_OVF1_vect){
	g_timeToLeft++;
	if (g_timeToLeft > NUM_OF_11S_CYCLES)
	{
		STOP_ON_TIMER();
		wdt_enable(1);
		while(1);
	}
	
}

int main(void)
{
	
	wdt_disable();
	INIT_DIGIT_SELECT_PORT();
	INIT_SEGMENT_PORT();
	INIT_TIMER();
	
 	INIT_PERIPHERIAL_ON_PORT();
	INIT_RADIO_PORT();
	
	ON_PERIPHERIA();
	_delay_ms(200);
	
	ow_reset();
	ow_command( DS18X20_CONVERT_T, 0 );
	
	sei();
	
	_delay_ms(780);
	while(1){
		
		uint8_t diff = OW_SEARCH_FIRST;
		diff = ow_rom_search( diff, sensorId );
		DS18X20_read_temp(sensorId,&g_temp);
		g_selectSensor = 0;
		SendTempDataToBase();
		
		if (diff !=  OW_LAST_DEVICE)//Если есть еще датчики то читаем от него данные
		{
			if (g_cyclesToSleep == 0)
			{
				_delay_ms(3);
			}else
				_delay_ms(TIME_TO_DISP_SENSOR);
			diff = ow_rom_search( diff, sensorId );
			DS18X20_read_temp(sensorId,&g_temp);
			g_selectSensor = 1;
			SendTempDataToBase();
		}
		
		
		
		g_timerToSleep++;//Если оттикали количество секунд то уходим в сон
		if (~(g_timerToSleep >= g_cyclesToSleep))//если не оттикали нужное количество циклов то перезапускаем измерение и ждем
		{	
			ow_reset();
			ow_command( DS18X20_CONVERT_T, 0 );
			_delay_ms(TIME_TO_DISP_SENSOR);
			
		}else{//если оттикали то уходим в глубокий сон
			g_timerToSleep = 0;
			STOP_TIMER();
			OFF_PERIPHERIA();
			DEINIT_DIGIT_SELECT_PORT();
			DEINIT_SEGMENT_PORT();
			g_cyclesToSleep = 0;
			//Настраиваем вход прерывания 0 как вход и разрешаем прерывание
			ENABLE_INT0();
			START_ON_TIMER();
			//Уходим в глубокий сон
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}


	}

}