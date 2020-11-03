#include "MKL25Z4.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "audio.h"
#include "init.h"
#pragma once
osThreadId_t tBrainThreadId, tLedThreadId, tAudioThreadId, tMotorThreadId;
osMessageQueueId_t tLedMsgQueId, tAudioMsgQueId, tBrainMsgQueId, tMotorMsgQueId;

#define CONNECTION_START 6
#define MAZE_END 5
#define ROBOT_FORWARD 1
#define ROBOT_BACKWARD 2
#define ROBOT_LEFT 3
#define ROBOT_RIGHT 4
#define ROBOT_STOP 0

uint32_t AUDIO_START = 0;
uint32_t AUDIO_STOP = 1;
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

typedef struct
{
	uint8_t cmd;
	uint8_t data;
} dataPacket;

dataPacket rxData;

//DONE
void UART1_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(UART1_IRQn);
	
	if (UART1->S1 & UART_S1_RDRF_MASK)
	{
		//dataPacket = UART1->D;
	}
}

void tBrain (void *argument) {
	for (;;) {
		/*if(dataPacket == CONNECTION_START) {
			osMessageQueuePut(tAudioMsgQueId, &AUDIO_STOP, NULL, 0);
			osMessageQueuePut(tLedMsgQueId, LED_CONNECTED, NULL, 0);
			osMessageQueuePut(tMotorMsgQueId, &rxData, NULL, 0);
		} else if (dataPacket == MAZE_END) {
			osMessageQueuePut(tAudioMsgQueId, &dataPacket, NULL, 0);
			osMessageQueuePut(tLedMsgQueId, &dataPacket, NULL, 0);
		} else if (dataPacket == ROBOT_FORWARD) {
			
		} else if (dataPacket == ROBOT_BACKWARD) {
			
		} else if (dataPacket == ROBOT_LEFT) {
		
		} else if (dataPacket == ROBOT_RIGHT) {
		
		} else if (dataPacket == ROBOT_STOP) {
			
		} else {
		
		}*/
	}
}

void tLed (void *argument) {
}

void tMotor (void *argument) {
	for(;;) {
		/*if(rx_data==0)
		{
			TPM0_C0V = 0 / 1.2;
			TPM0_C1V = 0 / 1.2;
			TPM0_C2V = 0 / 1.2;
			TPM0_C3V = 0 / 1.2;
		}
		else if(rx_data==1)
		{
			TPM0_C0V = Freq_MOD(100) / 2;
			TPM0_C1V = Freq_MOD(100) / 2;
			TPM0_C2V = 0 / 1.2;
			TPM0_C3V = 0 / 1.2;
		}
		else if(rx_data==2)
		{
			TPM0_C0V = Freq_MOD(100) / 2;
			TPM0_C1V = Freq_MOD(100) / 2;
			TPM0_C2V = Freq_MOD(100) / 2;
			TPM0_C3V = Freq_MOD(100) / 2;
		}
		else if(rx_data==3)
		{
			TPM0_C0V = Freq_MOD(100) / 2;
			TPM0_C1V = Freq_MOD(100) / 1.5;
			TPM0_C2V = Freq_MOD(100) / 2;
			TPM0_C3V = Freq_MOD(100) / 1.5;
		}
		else if(rx_data==4)
		{
			TPM0_C0V = Freq_MOD(100) / 1.5;
			TPM0_C1V = Freq_MOD(100) / 2;
			TPM0_C2V = Freq_MOD(100) / 1.5;
			TPM0_C3V = Freq_MOD(100) / 2;
		}*/
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
	//dataPacket = 0;
	SystemCoreClockUpdate();
	InitGPIO();
	Init_UART1(BAUD_RATE);
	InitPWM();
	InitBuzzer();
	
	osKernelInitialize();
	tBrainThreadId = osThreadNew(tBrain, NULL, NULL);
	tMotorThreadId = osThreadNew(tMotor, NULL, NULL);
	tLedThreadId = osThreadNew(tLed, NULL, NULL);
	tAudioThreadId = osThreadNew(tAudio, NULL, NULL);
	
	tBrainMsgQueId = osMessageQueueNew(msgCount, sizeof(dataPacket), NULL);
	tMotorMsgQueId = osMessageQueueNew(msgCount, sizeof(dataPacket), NULL);
	tLedMsgQueId = osMessageQueueNew(msgCount, sizeof(dataPacket), NULL);
	tAudioMsgQueId = osMessageQueueNew(msgCount, sizeof(dataPacket), NULL);
	osKernelStart();
	for(;;);
}
