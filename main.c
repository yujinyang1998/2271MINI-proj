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

#define AUDIO_START 0
#define AUDIO_STOP 1
#define AUDIO_MOVE 2
#define LED_STATIONARY 0
#define LED_MOVE 1
#define LED_CONNECTED 2

#define FREQ_MOD(x) (375000/x)
#define BAUD_RATE 9600
#define CMDMASK (3 << 6)
#define DATAMASK (0b111111)

osEventFlagsId_t dataFlag;						// To Run tBrain when new data is received
osEventFlagsId_t motorFlag;						// To Run tControl after decoding of data
osEventFlagsId_t special_event_flag;	// For Connection and ending buzzer
osEventFlagsId_t init_light_flag;  		// Only raised when bluetooth start
osEventFlagsId_t init_audio_flag;  		// Only raised when bluetooth start

volatile unsigned char rxData = 0;
volatile uint8_t dir = 0;


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
		if (rxData & (1<<5)) {
			osEventFlagsSet(special_event_flag, 0x0004);
		}
		
		if (rxData & 0b10000) {
			dir |= (rxData & 0b00011111);
		} else {
			dir = 0;
		}
		
		osEventFlagsSet(motorFlag, 0x0001);
	}
}

void tLed (void *argument) {
	for(;;) {
		PTE->PDOR = MASK(PTE2_PIN);
		PTB->PDOR = MASK(PTB11_PIN);
	}
}

void tMotor (void *argument) {
	for(;;) {
		osEventFlagsWait(motorFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		move(dir);
	}
}

void tAudio (void *argument) {
	int thisNote = 0;

	for(; ;) {
			int noteDuration = 750 / noteDurations[thisNote];
			int freq = 375000 / melody[thisNote];
			TPM1->MOD = freq;
			TPM1_C0V =  0.05*freq;
			osDelay(noteDuration*0.3);
			int pauseBetweenNotes = noteDuration * 1;
			osDelay(pauseBetweenNotes*0.3);
			TPM1->MOD=0;
			TPM1_C0V = 0;
			
			if(thisNote<112){ 
				thisNote++;
			} else {
				thisNote = 0;
			}
	}
}

int main(void)
{
	SystemCoreClockUpdate();
	InitLed();
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
