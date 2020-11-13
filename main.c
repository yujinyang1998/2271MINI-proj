#include "MKL25Z4.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "audio.h"
#include "init.h"
#include "motor.h"
#include "led.h"
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

osEventFlagsId_t dataFlag;						// Trigger when new data is recieved
osEventFlagsId_t motorFlag;						// Trigger when tBrain processes data
osEventFlagsId_t startEndFlag;	      // Trigger when connected start and end.
osEventFlagsId_t ledFlag;  		        // Trigger when connected to start the LED sequence
osEventFlagsId_t audioFlag;  		      // Trigger when connected to play the audio

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
		
		if (rxData == 32 | rxData == 79) {
			osEventFlagsSet(startEndFlag, 0x0001);
		}
		
		if (rxData & 0b10000) {
			dir |= (rxData & 0b1111);
		} else {
			dir &= ~(rxData & 0b1111);
		}
		
		osEventFlagsSet(motorFlag, 0x0001);
	}
}

void tLed (void *argument) {
	int i = 0;
	int flashMask = 0xFF;
	osEventFlagsWait(ledFlag, 0x0001, osFlagsWaitAny, osWaitForever);
	
	for(int j = 0; j < 2; j++) {
		GreenLED(0xFF);
		osDelay(500);
		GreenLED(0);
		osDelay(500);
	}
	
	for(;;) {
		if(dir == 0) {
			RedLED(flashMask);
			GreenLED(0xFF);
			osDelay(250);
		} else{
			GreenLED(MASK(i));
			RedLED(flashMask);
			osDelay(500);
			if(i == 7){
				i = 0;
			} else {
				i++;
			}
		}
		flashMask = ~flashMask;
	}
}

void tMotor (void *argument) {
	for(;;) {
		osEventFlagsWait(motorFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		move(dir);
	}
}

void tStartEnd (void *argument) {
	for(;;) {
		osEventFlagsWait(startEndFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		if (rxData == 32){
			for(int i = 0; i < startSize; i++) {
				int freq = 375000 / start[i];
				TPM1->MOD = freq;
				TPM1_C0V =  0.05*freq;
				osDelay(150);
			}
			osEventFlagsSet(ledFlag, 0x0001);
			osEventFlagsSet(audioFlag, 0x0001);
		} else if (rxData == 79) {
			for(int i = 0; i < stopSize; i++) {
				osDelay(150);
				int freq = 375000 / stop[i];
				TPM1->MOD = freq;
				TPM1_C0V =  0.05*freq;
			}
		}
	}
}

void tAudio (void *argument) {
	for(; ;){
			osEventFlagsWait(audioFlag, 0x0001, osFlagsWaitAny, osWaitForever);
			int thisNote = 0;
			for(; ;) {
			if(rxData == 79) {
				break;
			} else {
				int freq = 375000 / melody[thisNote];
				TPM1->MOD = freq;
				TPM1_C0V =  0.05*freq;
				osDelay(100);
					if(thisNote<112){ 
						thisNote++;
					} else {
						thisNote = 0;
					}
				TPM1->MOD=0;
				TPM1_C0V = 0;
				osDelay(100);
				}
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
	startEndFlag = osEventFlagsNew(NULL);
	ledFlag = osEventFlagsNew(NULL);
	audioFlag = osEventFlagsNew(NULL);
	
	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tMotor, NULL, NULL);
	osThreadNew(tLed, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);
	osThreadNew(tStartEnd, NULL, NULL);

	osKernelStart();
	for(;;){}
}