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
#include "ds18x20.h"
#include "onewire.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

#define CYCLES_BEFORE_SLEEP 3
char g_cyclesToSleep = 0;
char g_timerToSleep = 0;

enum state {MEASURE_AND_TRANSMIT,MEAS_AND_DISP,GO_TO_SLEEP,GO_TO_DEEP_SLEEP};
state g_state;

signed char g_temp = 0,g_selectSensor = 0;

#define MAXSENSORS 2
uint8_t gSensorIDs[MAXSENSORS][8];
	
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

#define DIGIT_SELECT_PORT PORTD
#define DIGIT_SELECT_DDR  DDRD
#define DIGIT_SELECT_PIN  PIND

#define DIGIT_FIRST_PIN		PD6
#define DIGIT_SECOND_PIN	PD5
#define DIGIT_THIRD_PIN		PD4

#define DIGIT_PORT_MASK ( _BV(DIGIT_FIRST_PIN) | _BV(DIGIT_SECOND_PIN) | _BV(DIGIT_THIRD_PIN) )
#define INIT_DIGIT_SELECT_PORT()	(DIGIT_SELECT_DDR |= (DIGIT_PORT_MASK));\
									(DIGIT_SELECT_PORT &= ~DIGIT_PORT_MASK);
									
#define DEINIT_DIGIT_SELECT_PORT() (DIGIT_SELECT_DDR &= ~(DIGIT_PORT_MASK));\
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

ISR(TIMER0_OVF0_vect){
	DynamicIndication(g_temp,g_selectSensor);
}

#define PERIPHERIAL_ON_PORT PORTD
#define PERIPHERIAL_ON_DDR  DDRD
#define PERIPHERIAL_ON_PIN  PD0

#define INIT_PERIPHERIAL_ON_PORT()	PERIPHERIAL_ON_DDR |= _BV(PERIPHERIAL_ON_PIN);

#define ON_PERIPHERIA()				PERIPHERIAL_ON_PORT |=  _BV(PERIPHERIAL_ON_PIN);

#define OFF_PERIPHERIA()			PERIPHERIAL_ON_PORT &=  ~_BV(PERIPHERIAL_ON_PIN);



#define ENABLE_INT0() DDRD |=  _BV(PD2);\
					  PORTD |= _BV(PD2);\
					  GIMSK |= _BV(INT0);
#define DISABLE_INT0() GIMSK &= ~_BV(INT0);


#define START_ON_TIMER() TCCR1A = 0;\
		TCCR1B =  (1<<CS12) | (0<<CS11) | (1<<CS10);\
		TIMSK = (1<<TOIE1);\
		TCNT1 = 0x0000;
		
#define STOP_ON_TIMER() TCCR1B = 0;\
		TIMSK = 0;
		
#define  NUM_OF_11S_CYCLES 1
int g_timeToLeft = 0;

ISR(INT0_vect){
		DISABLE_INT0();
		STOP_ON_TIMER();
		wdt_disable();
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
		g_state = MEAS_AND_DISP;
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




ISR(TIMER1_OVF1_vect){
	g_timeToLeft++;
	if (g_timeToLeft > NUM_OF_11S_CYCLES)
	{
		STOP_ON_TIMER();
		wdt_enable(0xFF);
		g_state = GO_TO_DEEP_SLEEP;
	}
	g_state = GO_TO_SLEEP;
	
}


static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
//	uart_puts_P( NEWLINESTR "Scanning Bus for DS18X20" NEWLINESTR );
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
		//	uart_puts_P( "No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
		//	uart_puts_P( "Bus Error" NEWLINESTR );
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	
	return nSensors;
}

int main(void)
{
	
	wdt_disable();

	g_state = MEASURE_AND_TRANSMIT;
	INIT_RADIO_PORT();
	INIT_PERIPHERIAL_ON_PORT();
	ON_PERIPHERIA();
	_delay_ms(500);
	
	ow_reset();
	_delay_ms(500);
	ow_command( DS18X20_CONVERT_T, 0 );
	char numOfSensors = search_sensors();
	//g_cyclesToSleep = CYCLES_BEFORE_SLEEP;
	sei();
	
	_delay_ms(780);
	while(1){

		switch(g_state){
			case MEAS_AND_DISP://Таймер для отображения цифр, порт для вывода цифр и количество показов цифр устанавливаются в INT0
			case MEASURE_AND_TRANSMIT:{
				cli();
				DS18X20_read_temp(gSensorIDs[0],&g_temp);
				sei();
				g_selectSensor = 0;
				SendTempDataToBase();
		
				if (numOfSensors > 0)//Если есть еще датчики то читаем от него данные
				{
					if (g_cyclesToSleep == 0)
					{
						_delay_ms(3);
					}else
					_delay_ms(TIME_TO_DISP_SENSOR);
					cli();
					DS18X20_read_temp(gSensorIDs[1],&g_temp);
					sei();
					g_selectSensor = 1;
					SendTempDataToBase();
				}
			}break;
			case GO_TO_SLEEP:{//засыпаем по флагу сна				
				STOP_TIMER();
				OFF_PERIPHERIA();
				DEINIT_DIGIT_SELECT_PORT();
				DEINIT_SEGMENT_PORT();
				g_cyclesToSleep = 0;
				//Настраиваем вход прерывания 0 как вход и разрешаем прерывание
				ENABLE_INT0();
				START_ON_TIMER();
				//Спим не глубоко а просыпаемся на тикающий таймер для просыпания и отправки температуры по радиоканалу
				set_sleep_mode(SLEEP_MODE_IDLE);
				sleep_mode();
				//ON_PERIPHERIA();
			}break;
			case GO_TO_DEEP_SLEEP:{//уходим в глубокий сон по истечению таймера перед отправкой температуры при взведенной собаке
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				sleep_mode();
			}break;
		}
		
		
		
		
		g_timerToSleep++;//Если оттикали количество секунд то уходим в сон
		if ((g_timerToSleep < g_cyclesToSleep))//если не оттикали нужное количество циклов то перезапускаем измерение и ждем
		{	cli();
			ow_reset();
			ow_command( DS18X20_CONVERT_T, 0 );
			sei();
			_delay_ms(TIME_TO_DISP_SENSOR);
			
		}else{
			g_timerToSleep = 0;
			g_state = GO_TO_SLEEP;
		}


	}

}