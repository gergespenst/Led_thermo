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

#define SEGMENT_PORT (PORTB)
#define SEGMENT_DDR  DirReg(&SEGMENT_PORT)
#define SEGMENT_PIN  PinReg(&SEGMENT_PORT)

#define INIT_SEGMENT_PORT()  (SEGMENT_DDR = 0xFF);\
							 (SEGMENT_PORT = 0xFF);
							
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

signed char g_temp,g_selectSensor = 0;
ISR(TIMER0_OVF0_vect){
	DynamicIndication(g_temp,g_selectSensor);
}


#include "ds18x20.h"
#include "onewire.h"

uint8_t sensorId[8];

int main(void)
{
	ow_reset();
	ow_command( DS18X20_CONVERT_T, 0 );
	INIT_DIGIT_SELECT_PORT();
	INIT_SEGMENT_PORT();
	INIT_TIMER();
	
	sei();

	g_temp = 0;

	_delay_ms(780);
	while(1){
		uint8_t diff = OW_SEARCH_FIRST;
		diff = ow_rom_search( diff, sensorId );//DS18X20_find_sensor( &diff, onewire_enum );
		DS18X20_read_temp(sensorId,&g_temp);
		g_selectSensor = 0;
		
		
		if (diff !=  OW_LAST_DEVICE)
		{
		_delay_ms(TIME_TO_DISP_SENSOR);
	    diff = ow_rom_search( diff, sensorId );//DS18X20_find_sensor( &diff, onewire_enum );
		DS18X20_read_temp(sensorId,&g_temp);
		g_selectSensor = 1;
		}
		
		ow_reset();
		ow_command( DS18X20_CONVERT_T, 0 );
		_delay_ms(TIME_TO_DISP_SENSOR);



	}

}