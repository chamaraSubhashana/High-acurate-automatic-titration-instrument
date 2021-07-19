/*
 * AutomaticTitration.c
 *
 * Created: 19-Apr-19 9:46:57 AM
 * Author : chamara Subhashana Silva
 */ 


/*=========== Pin Config ===============

	stepperMotor 
		PIN0 = PB2
		PIN1 = PB3
		PIN2 = PB4
		PIN3 = PB5

	Burrette LED and LDR reading = PD6
	Beaker LED and LDR reading = PD5
	
	start/stop button = PD3
	
	limit switch = PD2
	
	serial display data = PB0
	serial display clock = PB7
	
	burrette Sensor power = PA0
	beaker Sensor power = PD0
	PD1
	
	LDR Comparator Input = PB1
		
	Stirrer = PA1
	Solenoid = PD4

*/


/*++++++++++ Includes ++++++++++++*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
/*++++++++++ Definitions ++++++++++++*/
/*--- Stepper Motor Pin Config---*/
#define STEP_0 0x14
#define STEP_1 0x24
#define STEP_2 0x28
#define STEP_3 0x18
/*-- General Definitions ---*/
#define HIGH 1 
#define LOW  0

#define DATA PB0
#define CLK  PB7
/*--  7_segment Definitions---*/
#define ZERO	0xEE
#define ONE		0x88
#define TWO		0x6D
#define THREE	0xCD
#define FOUR	0x8B
#define FIVE	0xC7
#define SIX		0xE7
#define SEVEN	0x8C
#define EIGHT	0xEF
#define NINE	0xCF


#define DISP_3 0x11
#define DISP_0 0x22
#define DISP_1 0x84
#define DISP_2 0x08

/*--- Machine States ---*/
#define IDLE			0  // Wait for start Press
#define INITIALIZE		1  // Get initial Level 
#define	TITRATION_START 2  // Open Solenoid and wait for color change
#define	TITRARION_DONE	3  // get the final Level
#define DISPLAY_RESULT	4  // Calculate and Display

/*++++++++++ Enums and Structures ++++++++++++*/
typedef enum _MOTORCTRL{
	
	UP,DWN,STOP
	
}motorCtrl;

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
bool isAcc = true;
/*++++++++++ SubRoutines ++++++++++++*/
bool isBitSet(uint8_t Register, uint8_t bit){
	
	bool ret = false;
	Register &= (1<<bit);
	if(Register != 0){
		ret = true;
	}
	return ret;
}

/*--- Pheripheral Control Subs---*/
void motorFunc(void){
	switch(direction){
		case DWN:
		stepCounter++;
		switch(Status){
			case 0:
			PORTB = STEP_0;
			Status =1;
			break;
			case 1:
			PORTB = STEP_1;
			Status =2;
			break;
			case 2:
			PORTB = STEP_2;
			Status =3;
			break;
			case 3:
			PORTB = STEP_3;
			Status =0;
			break;
			default:
			break;
		}
		
		break;
		case UP:
		stepCounter++;
		switch(Status){
			case 0:
			PORTB = STEP_3;
			Status =1;
			break;
			case 1:
			PORTB = STEP_2;
			Status =2;
			break;
			case 2:
			PORTB = STEP_1;
			Status =3;
			break;
			case 3:
			PORTB = STEP_0;
			Status =0;
			break;
			default:
			break;
		}
		break;
		case STOP:
		PORTB = 0x00;
		break;
		
		default:
		break;
	}
	
	
}
void burretteSensorOn(void){
	portStatus = PORTA&0x02;
	PORTA = (1<<PORTA0) | portStatus;
	}
void burretteSensorOff(void){
	portStatus = PORTA&0x02;
	PORTA = (0<<PORTA0) | portStatus;
	}
void beakerSensorOn(void){
	portStatus = PORTD&0xFE;
	PORTD = (1<<PORTD0) |portStatus;
	}
void beakerSensorOff(void){
	portStatus = PORTD&0xFE;
	PORTD = (0<<PORTD0) |portStatus;
	}
void stirrerOn(void){
		enableStirrer = true; // Stirrer Control
	}
void stirrerOff(void){
		enableStirrer = false; // Stirrer Control
	}
void solenoidOn(void){
	portStatus = PORTD&0xEF;
	PORTD |= (1<<PORTD4) | portStatus;
}
void solenoidOff(void){
	portStatus = PORTD&0xEF;
	PORTD |= (0<<PORTD4) | portStatus;
}	

void initStepCounter()
{
	stepCounter = 0;
}
int getSteps(){
	return stepCounter;	
}

/*-- Functions for Display Update--*/

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
	
	uint8_t mask = 0x01;
	for(int i = 0; i < 8;i++){
		portStatus = PORTB&0x7E;
		PORTB = (LOW<<CLK) | portStatus;
		uint8_t val = (byte&(mask<<i));		
		if(val != 0x00){
			PORTB = (HIGH<<DATA) | portStatus;	
			PORTB = (HIGH<<CLK) | (HIGH<<DATA)| portStatus;
			PORTB = (LOW<<CLK) |(HIGH<<DATA)| portStatus;
			PORTB = (LOW<<DATA)| portStatus;	
		}
		else
		{
				PORTB = (LOW<<DATA)| portStatus;
				PORTB = (HIGH<<CLK) | portStatus;
				PORTB = (LOW<<CLK) | portStatus;
		}
	}
	PORTB = (LOW<<CLK)| portStatus;
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
		portStatus = PORTB&0x7E;
		PORTB = (HIGH<<CLK)| portStatus;
		
		
}  



/*++++++++++ ISRs ++++++++++++*/

// Stepper Motor Control
ISR(TIMER0_OVF_vect){	
	//updateDisp();
	
}	
ISR(TIMER0_COMPA_vect){
	TCNT0 = 0x00;
 		if(isAcc == true){
 			if(OCR0A >=0x20){
 				OCR0A -= 1;			
 			}
 		}else{
 			if(OCR0A <= 0x60){
 				OCR0A += 1;
 			}else{
				 direction = STOP;
			}		
 		}
 		
		
		motorFunc();	
}	

ISR(INT1_vect){
	switch(stt)
	{
		case IDLE:
			stt = INITIALIZE;
		break;
		default:
		break;
	}
	
} 
 
//limit switches
ISR(INT0_vect){
	if(!isBitSet(PIND,2))
	{
		if(direction == UP){
		direction = DWN;
		}
		else if(direction == DWN){
			direction = UP;
		}
	}
	else
	{
		direction = STOP;
	}
}
	

// Stirrer  Control
ISR(TIMER1_OVF_vect){
	portStatus = PORTA&0x01;
	PORTA = (0<<PORTA1) | portStatus;	
	
}
ISR(TIMER1_COMPB_vect){
	if(true == enableStirrer){
	portStatus = PORTA&0x01;
	PORTA = (1<<PORTA1) | portStatus;
		}
}

ISR(ANA_COMP_vect){
	
}

// Level Sensing and color change Sensing


/*++++++++++ RESET ++++++++++++*/
void RESET(void){
	
	/* Setting port Data Directions */
	DDRB   =	0xFC;
	DDRA   =	0x03;
	DDRD   =	0x01;
	/* Enable Timer Interrupts */
	TIMSK  =	0xA3;
	TCCR1B =	0x02;
	TCCR0B =	0x03; 
	MCUCR  = (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00); // External interrupt enabled EX_INT_1 and EX_INT_0
	GIMSK  = (1<<INT1) | (1<<INT0);
	OCR1BH =	0x7F;
	OCR1BL =	0x00;
	UBRRL  =	0x19;
	ACSR   = (1<<ACBG) | (1<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
	OCR0A = 0x60;
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
}

/*++++++++++ MAIN ++++++++++++*/
int main(void)
{
	RESET();
	Init();
	stt = IDLE;
isAcc = true;
   
   
	while (1) 
    {
		
		switch(stt){
			
			case IDLE:
						if(!isBitSet(ACSR,ACO)){
							isAcc = false;
							stt = INITIALIZE;
						}
						else{
							isAcc = true;
						}
			break;
			
			case INITIALIZE:
				if(direction == STOP){
					
					direction = UP;
					isAcc = true;
				}
				else{
					if(isBitSet(ACSR,ACO)){
						direction = STOP;
						stt = TITRATION_START;
					}
						
					
				}
				
				
			break;
			
			case TITRATION_START:
			
				
			
			break;
		
			case TITRARION_DONE:
		
				
			break;
			
			case DISPLAY_RESULT:
				
			
			break;
			
		}
		
		
    }
}

