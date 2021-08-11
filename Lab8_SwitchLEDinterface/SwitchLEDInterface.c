// ***** 0. Documentation Section *****
// SwitchLEDInterface.c for Lab 8
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to toggle an LED
// while a button is pressed and turn the LED on when the
// button is released.  This lab requires external hardware
// to be wired to the LaunchPad using the prototyping board.
// January 15, 2016
//      Jon Valvano and Ramesh Yerraballi

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
unsigned long in;
unsigned long led;

#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
//#define GPIO_PORTE_CR_R 				(*((volatile unsigned long *)0x40024524))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
void Delay100ms(unsigned long time){
	unsigned long i;
	while(time > 0){
		i = 1333333;  // this number means 100ms
		while(i > 0){
			i = i - 1;
   }
   time = time - 1; // decrements every 100 ms
 }
}
// PE0, PB0, or PA2 connected to positive logic momentary switch using 10k ohm pull down resistor
// PE1, PB1, or PA3 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).
int main(void){ volatile unsigned long delay;
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  TExaS_Init(SW_PIN_PE0, LED_PIN_PE1, ScopeOn);  // activate grader and set system clock to 80 MHz
  //1) Make PE1 an output and make PE0 an input.
	//2) The system starts with the LED on (make PE1 =1).
	//3) Wait about 100 ms
	//4) If the switch is pressed (PE0 is 1), then toggle the LED once, else turn the LED on.
	//5) Steps 3 and 4 are repeated over and over.
	
	SYSCTL_RCGC2_R |= 0x10;           // Port E clock
  delay = SYSCTL_RCGC2_R;           // wait 3-5 bus cycles
  GPIO_PORTE_DIR_R |= 0x02;         // PE1 output
  GPIO_PORTE_DIR_R &= ~0x01;        // PE0 input
  GPIO_PORTE_AFSEL_R &= ~0x17;      // not alternative
  GPIO_PORTE_AMSEL_R &= ~0x17;      // no analog
  GPIO_PORTE_PCTL_R &= ~0x000F0FFF; // bits for PE4,PE2,PE1,PE0
  GPIO_PORTE_DEN_R |= 0x03;         // enable PE1,PE0 
  EnableInterrupts();           // enable interrupts for the grader
	
	while(1){
		in = GPIO_PORTE_DATA_R & 0x01;
		if(in){
			GPIO_PORTE_DATA_R ^= 0x02;
			Delay100ms(1);
		}		
		else
			GPIO_PORTE_DATA_R = 0x02;
  }
	
}
