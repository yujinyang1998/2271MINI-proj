#include "MKL25Z4.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "audio.h"
#include "init.h"
#include "motor.h"
#pragma once

#define CONNECTION_START 6
#define MAZE_END 5
#define ROBOT_FORWARD 1
#define ROBOT_BACKWARD 2
#define ROBOT_LEFT 3
#define ROBOT_RIGHT 4
#define ROBOT_STOP 0

#define AUDIO_START 0
#define AUDIO_STOP 1
#define AUDIO_MOVE 2
#define LED_STATIONARY 0
#define LED_MOVE 1
#define LED_CONNECTED 2

#define FREQ_MOD(x) (375000/x)
#define BAUD_RATE 9600
#define PTD0_Pin 0 	//Front Left
#define PTD1_Pin 1	//Front Right
#define PTD2_Pin 2	//Back Left
#define PTD3_Pin 3	//Back Right
#define PTC1_Pin 1  //buzzer
#define PTC2_Pin 2  //buzzer
#define MASK(x) (1 << (x))
#define CMDMASK (3 << 6)
#define DATAMASK (0b111111)

osEventFlagsId_t dataFlag;						// To Run tBrain when new data is received
osEventFlagsId_t motorFlag;						// To Run tControl after decoding of data
osEventFlagsId_t special_event_flag;	// For Connection and ending buzzer
osEventFlagsId_t init_light_flag;  		// Only raised when bluetooth start
osEventFlagsId_t init_audio_flag;  		// Only raised when bluetooth start

volatile unsigned char rxData = 0;
volatile uint8_t dir = 0;

#define PTB8_PIN 8
#define PTB9_PIN 9
#define PTB10_PIN 10
#define PTB11_PIN 11
#define PTE2_PIN 2
#define PTE3_PIN 3
#define PTE4_PIN 4
#define PTE5_PIN 5

void InitLed(void) {
	// Configure MUX settings to make pins to ALT3: TPM1_CH0 and TPM1_CH1
  // Enable Clock to PORTB and PORTE
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK));
  
  // Configure Mode 1 for PWM pin operation
  // Alternative 1 GPIO
  PORTB->PCR[PTB8_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB8_PIN] |= PORT_PCR_MUX(1);
  PORTB->PCR[PTB9_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB9_PIN] |= PORT_PCR_MUX(1);
	PORTB->PCR[PTB10_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB10_PIN] |= PORT_PCR_MUX(1);
	PORTB->PCR[PTB11_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB11_PIN] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE2_PIN] |= PORT_PCR_MUX(1);
	PORTE->PCR[PTE3_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE3_PIN] |= PORT_PCR_MUX(1);
	PORTE->PCR[PTE4_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE4_PIN] |= PORT_PCR_MUX(1);
	PORTE->PCR[PTE5_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE5_PIN] |= PORT_PCR_MUX(1);
}

//DONE
void UART1_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(UART1_IRQn);
	
	if (UART1->S1 & UART_S1_RDRF_MASK)
	{
		rxData = UART1->D;
		osEventFlagsSet(dataFlag, 0x0001);
	}
}

void tBrain (void *argument) {
	for(;;) {
		osEventFlagsWait(dataFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		osEventFlagsClear(dataFlag, 0x0001);
		if (rxData >> 6 == 1) {
			osEventFlagsSet(special_event_flag, 0x0004);
		}
		if (rxData >> 5 == 1) {
			dir |= (rxData & 0b11111);
		} else {
			dir &= ~(rxData & 0b11111);
		}
		
		osEventFlagsSet(motorFlag, 0x0001);
	}
}

void tLed (void *argument) {
	for(;;) {
		
	}
}

void tMotor (void *argument) {
	for(;;) {
		
	}
}

void tAudio (void *argument) {
	int thisNote = 0;

	for(; ;) {
			int noteDuration = 750 / noteDurations[thisNote];
			int freq = 375000 / melody[thisNote];
			TPM0->MOD = freq;
			TPM0_C1V =  0.05*freq;
			osDelay(noteDuration*0.8);
			int pauseBetweenNotes = noteDuration * 1;
			osDelay(pauseBetweenNotes*0.8);
			TPM0->MOD=0;
			TPM0_C1V = 0;
			
			if(thisNote<112){ 
				thisNote++;
			} else {
				thisNote = 0;
			}
	}
}

/* Delay Function */
static void delay(volatile uint32_t nof)
{
	while (nof != 0)
	{
		__asm("NOP");
		nof--;
	}
}

int main(void)
{
	uint32_t msgCount = 0;
	SystemCoreClockUpdate();
	InitGPIO();
	Init_UART1(BAUD_RATE);
	InitMotor();
	InitBuzzer();
	
	osKernelInitialize();
	motorFlag = osEventFlagsNew(NULL);
	dataFlag = osEventFlagsNew(NULL);
	special_event_flag = osEventFlagsNew(NULL);
	init_light_flag = osEventFlagsNew(NULL);
	init_audio_flag = osEventFlagsNew(NULL);
	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tMotor, NULL, NULL);
	osThreadNew(tLed, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);

	osKernelStart();
	for(;;){}
}
