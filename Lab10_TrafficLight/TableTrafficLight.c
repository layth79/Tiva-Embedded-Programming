// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control

#define SYSCTL_RIS_R            (*((volatile unsigned long *)0x400FE050))
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define SYSCTL_RCC_R            (*((volatile unsigned long *)0x400FE060))
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal Value
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // 6 MHz Crystal
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  // 8 MHz Crystal
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16 MHz Crystal
#define SYSCTL_RCC2_R           (*((volatile unsigned long *)0x400FE070))
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
                                            // MHz
#define SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2LSB  0x00400000  // Additional LSB for SYSDIV2
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}	
	
void SysTick_Wait(unsigned long delay1){
  NVIC_ST_RELOAD_R = delay1-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}

void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}
	
struct State {
  unsigned long OutT; 
	unsigned long OutP;
  unsigned long Time;  
  unsigned long Next[8];}; 
typedef const struct State STyp;
#define GoWest 0
#define SlowWest 1
#define GoSouth 2
#define SlowSouth 3
#define GoWalk 4
#define AllRed 5
#define WalkOff 6
#define AllRed2 7
#define WalkOff2 8

STyp FSM[9]={
	{0x0C, 0x02, 100, {0,0,1,1,1,1,1,1}}, //0010 PF1
	{0x14, 0x02, 50, {2,2,2,2,4,4,4,2}},	
	{0x21, 0x02, 100, {2,3,2,3,3,3,3,3}},
	{0x22, 0x02, 50, {0,0,0,0,4,4,4,4}},	//1000 PF3 
	{0x24, 0x08, 100, {4,5,5,5,4,5,5,5}},
	{0x24, 0x02, 25, {6,6,6,6,4,6,6,6}},
	{0x24, 0x00, 10, {7,7,7,7,4,7,7,7}},
	{0x24, 0x02, 25, {8,8,8,8,4,8,8,8}},
	{0x24, 0x00, 10, {0,0,2,0,4,0,2,0}}};
// ***** 3. Subroutines Section *****
unsigned long S;  // index to the current state 
unsigned long Input;

int main(void){ volatile unsigned long delay;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
  SysTick_Init();   // Program 10.2
	
  SYSCTL_RCGC2_R |= 0x32;      // 1) F B E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2, PE1, PE0 
  GPIO_PORTE_PCTL_R = 0x00000000; // 4) enable regular GPIO 
  GPIO_PORTE_DIR_R = 0x00;
  GPIO_PORTE_AFSEL_R = 0x00; 
	GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2, PE1, PE0 
	
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R = 0x00000000; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R = 0x00;
	GPIO_PORTB_DEN_R = 0x3F;    // 7) enable digital on PB5-0
	
	GPIO_PORTF_AMSEL_R &= ~0x0A; // 3) disable analog function on PF3, PF1
	GPIO_PORTF_PCTL_R = 0x00000000;
	GPIO_PORTF_DIR_R |= 0x0A; // 5) outputs on PF3, PF1
	GPIO_PORTF_AFSEL_R = 0x00; // 6) regular function on PF3, PF1 (technically all the bits)
	GPIO_PORTF_DEN_R = 0x0A; // 7) enable digital pins PF3, PF1 	
 	S = GoWest;
  EnableInterrupts();
  while(1){
		GPIO_PORTB_DATA_R = FSM[S].OutT;
		GPIO_PORTF_DATA_R = FSM[S].OutP;
    SysTick_Wait10ms(FSM[S].Time);
		Input = GPIO_PORTE_DATA_R;
    S = FSM[S].Next[Input];
  }
}

