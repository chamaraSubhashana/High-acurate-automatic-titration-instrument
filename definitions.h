/*
 * definitions.h
 *
 * Created: 05-May-19 10:50:27 AM
 *  Author: Chamara Subhashana Silva
 */ 


#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

/*=========== Pin Config ===============

	stepperMotor 
		PIN0 = PB4
		PIN1 = PB5
		PIN2 = PB6
		PIN3 = PB7

	
	start/stop button = PD3
	
	limit switch = PD2
	
	serial display data = PD1
	serial display clock = PD0
	
	burrete Sensor power = PB2
	beaker Sensor power = PB3

	
	LDR Comparator Input = PB1
		
	Stirrer = PA1
	Solenoid = PD4

*/


/*++++++++++ Definitions ++++++++++++*/
/*--- Stepper Motor Pin Config---*/
#define STEP_0 0xA0
#define STEP_1 0x60
#define STEP_2 0x50
#define STEP_3 0x90

/*-- General Definitions ---*/
#define HIGH 1 
#define LOW  0



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

/*--- Serial Display Drive Const (Do not Change) ---*/
#define DISP_3 0x80
#define DISP_2 0x48
#define DISP_1 0x22
#define DISP_0 0x11

#define DIS_PORT PORTD
#define DATA PORTD1
#define CLK  PORTD0

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

enum _speed{
	
	ACC,DEC,UNIFORM
	
}speed;


#endif /* DEFINITIONS_H_ */