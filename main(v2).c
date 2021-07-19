/*
 * AutomaticTitration.c
 *
 * Created: 19-Apr-19 9:46:57 AM
 * Author : chamara Subhashana Silva
 */ 


/*++++++++++ Includes ++++++++++++*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/delay.h>
#include "definitions.h"

/*++++++++++ Global Variables ++++++++++++*/

motorCtrl direction = STOP;
bool enableStirrer = false;
uint8_t btnDebounce = 0x00;
int Status;
uint8_t digitBuff[4];
uint8_t displayScan = 0;
uint8_t portStatus;
int stepCounter = 0;
uint8_t stt =0;
uint8_t numSteps = 0;
int reading = 0;

/*++++++++++ SubRoutines ++++++++++++*/

/*Bit functions*/
#pragma region BIT_FUNCTIONS

bool isBitSet(uint8_t Register, uint8_t bit){
	
	bool ret = false;
	Register &= (1<<bit);
	if(Register != 0){
		ret = true;
	}
	return ret;
}

#pragma endregion BIT_FUNCTIONS

/*Peripheral Control Subs*/
#pragma region OTHER_PHERIPHERAL_DRIVERS

void burretteSensorOn(void){
	
	PORTB |= (1<<PORTB2);
}
void burretteSensorOff(void){
	
	PORTB &= ~(1<<PORTB2);
}
void beakerSensorOn(void){
	
	PORTB |= (1<<PORTB3);
}
void beakerSensorOff(void){

	PORTB &= ~(1<<PORTB3);
}
void stirrerOn(void){
	enableStirrer = true; // Stirrer Control
}
void stirrerOff(void){
	enableStirrer = false; // Stirrer Control
}
void solenoidOn(void){
	
	PORTD |= (1<<PORTD4);
}
void solenoidOff(void){

	PORTD &= ~(1<<PORTD4);
}

#pragma endregion OTHER_PHERIPHERAL_DRIVERS

/*Motor Driver*/
#pragma region MOTOR_DRIVER

void motorFunc(void){
	PORTB &= 0x0f;
	switch(direction){
		case DWN:
		stepCounter++;
		switch(Status){
			case 0:
			PORTB |= STEP_0;
			Status =1;
			break;
			case 1:
			PORTB |= STEP_1;
			Status =2;
			break;
			case 2:
			PORTB |= STEP_2;
			Status =3;
			break;
			case 3:
			PORTB |= STEP_3;
			Status =0;
			break;
			default:
			break;
		}
		
		break;
		case UP:
		//stepCounter++;
		switch(Status){
			case 0:
			PORTB |= STEP_3;
			Status =1;
			break;
			case 1:
			PORTB |= STEP_2;
			Status =2;
			break;
			case 2:
			PORTB |= STEP_1;
			Status =3;
			break;
			case 3:
			PORTB |= STEP_0;
			Status =0;
			break;
			default:
			break;
		}
		break;
		case STOP:
		PORTB &= 0x0f;
		break;
		
		default:
		break;
	}
	
	
}
void initStepCounter()
{
	stepCounter = 0;
}
int getSteps(){
	return stepCounter;
}

#pragma endregion MOTOR_DRIVER

/*Display Driver*/
#pragma region DISPLAY_DRIVER
// Private
int toBCD(int value){
	
	int bcdResult = 0;
	int shift = 0;


	while (value > 0) {
		bcdResult |= (value % 10) << (shift++ << 2);
		value /= 10;
	}
	return bcdResult;
	
}

uint8_t getDigitImage(uint8_t digit){
	uint8_t ret = ZERO;
	
	switch(digit){
		case 0:
		ret = ZERO;
		break;
		case 1:
		ret = ONE;
		break;
		case 2:
		ret = TWO;
		break;
		case 3:
		ret = THREE;
		break;
		case 4:
		ret = FOUR;
		break;
		case 5:
		ret = FIVE;
		break;
		case 6:
		ret = SIX;
		break;
		case 7:
		ret = SEVEN;
		break;
		case 8:
		ret = EIGHT;
		break;
		case 9:
		ret = NINE;
		break;
		default:
		break;
	}
	return ret;
}

void transmit(uint8_t byte){
	
	uint8_t mask = 0x80;
	for(int i = 0; i < 8;i++){
		DIS_PORT &= ~(1<<CLK);
		uint8_t val = (byte&(mask>>i));
		
		if(val != 0x00){
			DIS_PORT |= (HIGH<<DATA);
			DIS_PORT |= (HIGH<<CLK) | (HIGH<<DATA);
			
		}
		else
		{
			DIS_PORT &= ~(1<<DATA) ;
			DIS_PORT |= (HIGH<<CLK) ;
		
		}
	}

	
}

// Used to Change the Digits when needed
void setDigits(int value){

	int BCD = toBCD(value);

	for(int i = 4 ; i > 0; i--){
		digitBuff[4-i] = getDigitImage(((BCD>>((i-1)*4))&0x0f));
	}


}

// Should called in timer overflow
void updateDisp(){
	
	switch(displayScan){
		case 0:
		transmit(digitBuff[0]);
		transmit(DISP_0);
		displayScan = 1;
		break;
		case 1:
		transmit(digitBuff[1]);
		transmit(DISP_1);
		displayScan = 2;
		break;
		case 2:
		transmit(digitBuff[2]);
		transmit(DISP_2);
		displayScan = 3;
		break;
		case 3:
		transmit(digitBuff[3]);
		transmit(DISP_3);
		displayScan = 0;
		break;
		
		
		default:
		break;
	}
	
	DIS_PORT |= (HIGH<<CLK);
	
	
}

#pragma endregion DISPLAY_DRIVER

/*++++++++++ ISRs ++++++++++++*/
#pragma region ISRs
ISR(TIMER0_OVF_vect){
	
}
 /*Motor Control and Display update*/
ISR(TIMER0_COMPA_vect){
		TCNT0 = 0x00;
	if(speed == ACC){
			if(OCR0A >=0x20){
				OCR0A -= 1;
			}
		}else if(speed == DEC){
			if(OCR0A <= 0x60){
				OCR0A += 1;
				}else{
				direction = STOP;
			}
		}else if(speed == UNIFORM){
		OCR0A = 0x60;
	}
	updateDisp();
	
	motorFunc();
}

/*Switch State Control*/
ISR(INT1_vect){
	switch(stt)
	{
		case IDLE:
			if(direction == STOP){
				burretteSensorOn();
				stt = INITIALIZE;
				direction = UP;
				speed = ACC;
				initStepCounter();
				
			}
		break;
		case INITIALIZE:
			
		break;
		default:
		break;
	}
}

//limit switches
ISR(INT0_vect){
	
	if(!isBitSet(PIND,PIND2))
	{
		if(direction == UP){
			direction = DWN;
			
			speed = ACC;
		}
		else if(direction == DWN){
			direction = UP;
			speed = ACC;
		}
	}
	else
	{
		
		direction = STOP;
	}
}


// Stirrer  Control
ISR(TIMER1_OVF_vect){
	PORTA &=  ~(1<<PORTA1) ;
}
ISR(TIMER1_COMPB_vect){
	if(true == enableStirrer){
		
		PORTA |= (1<<PORTA1);
	}
}

/* Level measurement*/
ISR(ANA_COMP_vect){
	
}

#pragma endregion ISRs

/*++++++++++ RESET ++++++++++++*/
void RESET(void){
	
	/* Setting port Data Directions */
	DDRB   =	0xFC;
	DDRA   =	0x03;
	DDRD   =	0x13;
	/* Enable Timer Interrupts */
	TIMSK  =	0xA3;
	TCCR1B =	0x03; 
	TCCR0B =	0x03; 
	PORTD = PORTD2;
	MCUCR  = (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00); // External interrupt enabled EX_INT_1 and EX_INT_0
	GIMSK  = (1<<INT1) | (1<<INT0);
	OCR1BH =	0x7F;
	OCR1BL =	0x00;
	UBRRL  =	0x19;
	ACSR   = (1<<ACBG) | (1<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
	OCR0A = 0x80;
	sei();
	
	
} 

/*++++++++++ INIT ++++++++++++*/
void Init(void){
	Status = 0; // Stepper motor Status parameter
	direction = DWN; // Stepper Direction UP,DWN or STOP
	
	// Switching off all the peripherals
	stirrerOff();
	beakerSensorOff();
	burretteSensorOff();
	solenoidOff();
	setDigits(0 );
}

/*++++++++++ MAIN ++++++++++++*/
int main(void)
{
	RESET();
	Init();
	stt = IDLE;
	speed = ACC;
  
	while (1) 
    { 
	switch(stt){
		case IDLE:
			// will come to rest position
			//beakerSensorOn();
			burretteSensorOn();
		break;
		
		case INITIALIZE:
			if(!isBitSet(ACSR,ACO)){
				
			}
			else{
				initStepCounter();
				setDigits(gets());
				direction = STOP;
				burretteSensorOff();
				stirrerOn();
				beakerSensorOn();			
				solenoidOn();
				stt = TITRATION_START;
				_delay_ms(2000);
			}
			
		break;
		
		case TITRATION_START:
			if(isBitSet(ACSR,ACO)){
					solenoidOff();
				stirrerOff();
				beakerSensorOff();
				burretteSensorOn();
				_delay_ms(1000);
				direction = DWN;
				stt =TITRARION_DONE;
			}
			else{
				
			}
		break;
		
		case TITRARION_DONE:
			if(!isBitSet(ACSR,ACO)){
				direction = STOP;
				setDigits(getSteps());
			}
			
		break;
		
		}
    }
}

