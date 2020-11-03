#include "MKL25Z4.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "audio.h"

osThreadId_t tBrainThreadId, tLedThreadId, tAudioThreadId, tMotorThreadId;
osMessageQueueId_t tLedMsgQueId, tAudioMsgQueId, tBrainMsgQueId, tMotorMsgQueId;

typedef struct
{
	uint8_t cmd;
	uint8_t data;
} dataPacket;

dataPacket rxData;

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

#define BAUD_RATE 9600
#define PTD0_Pin 0 	//Front Left
#define PTD1_Pin 1	//Front Right
#define PTD2_Pin 2	//Back Left
#define PTD3_Pin 3	//Back Right
#define PTC1_Pin 1  //buzzer
#define PTC2_Pin 2  //buzzer
#define MASK(x) (1 << (x))

void InitBuzzer(void) {
	// Configure MUX settings to make pins to ALT3: TPM1_CH0 and TPM1_CH1
  // Enable Clock Gating for PORTB
  SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
  
  // Configure Mode 3 for PWM pin operation
  // Alternative 3
  PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(4);
  
  PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(4);
  
  // Enable Clock Gating for Timer1
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
  
  // Select clock for TPM module
  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK or MCGPLLCLK/2
  
  // Set Modulo Value 20971520 / 128 = 163840 / 327 = 50 Hz
  //TPM1->MOD = 3276;
  
  // Set Modulo Value 48000000 / (128*50) = 50 Hz (50% duty cycle)
  TPM0->MOD = 7500; 
  
  // Edge-Aligned PWM
  // Update SnC register: CMOD = 01, PS=111 (128)
  TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
  TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
  TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
  
  // Enable PWM on TPM1 Channel 0 -> PTB0
  TPM0_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
  TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
  
  // Enable PWM on TPM1 Channel 1 -> PTB1
  TPM0_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
  TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}
	
//DONE
void Init_UART1(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;

	// enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// connect UART to pins for PTE0, PTE1
	PORTE->PCR[1] = PORT_PCR_MUX(3);
	PORTE->PCR[0] = PORT_PCR_MUX(3);

	// ensure txand rxare disabled before configuration
	UART1->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	// Set baud rate to 4800 baud
	divisor = bus_clock / (baud_rate * 16);
	UART1->BDH = UART_BDH_SBR(divisor >> 8);
	UART1->BDL = UART_BDL_SBR(divisor);

	// No parity, 8 bits, two stop bits, other settings;
	UART1->C1 = 0;
  UART1->S2 = 0;
  UART1->C3 = 0;
	
	NVIC_SetPriority(UART1_IRQn, 0);
	NVIC_ClearPendingIRQ(UART1_IRQn);
  NVIC_EnableIRQ(UART1_IRQn);
	
	UART1->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	UART1->C2 &= ~((UART_C2_TIE_MASK) | (UART_C2_RIE_MASK));
  UART1->C2 |= ((UART_C2_RIE_MASK));
	// Enable receiver
	UART1->C2 |= UART_C2_RE_MASK;
}

//DONE
void UART1_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(UART1_IRQn);
	
	if (UART1->S1 & UART_S1_RDRF_MASK)
	{
		//dataPacket = UART1->D;
	}
}


void InitGPIO(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));

}

int Freq_MOD(int freq)
{
	return 37500/freq;
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
		osDelay(noteDuration*0.4);
		int pauseBetweenNotes = noteDuration * 1;
		osDelay(pauseBetweenNotes*0.4);
		TPM0->MOD=0;
		TPM0_C1V = 0;
		if(thisNote<112){ 
			thisNote++;
		} else {
			thisNote = 0;
		}
	}
}
void InitPWM(void)
{
	// Configure MUX setting
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH0
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;		
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH1
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH2
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH3

	//Enable Clock gating for timers
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	//Set MOD to get freq of 100hz (random, motor control)
	TPM0->MOD = Freq_MOD(100);

	// Edge-Aligned PWM 
	// CMOD = 01 (count up), PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	//Motors
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
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
