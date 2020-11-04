#include "MKL25Z4.h" 
#define PTB0_Pin 0  //buzzer

void InitBuzzer(void) {
	// Configure MUX settings to make pins to ALT3: TPM1_CH0 and TPM1_CH1
  // Enable Clock Gating for PORTB
               // Device header
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  // Configure Mode 3 for PWM pin operation
  // Alternative 3
  PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
  
  // Enable Clock Gating for Timer1
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  
  // Select clock for TPM module
  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK or MCGPLLCLK/2
  
  // Set Modulo Value 20971520 / 128 = 163840 / 327 = 50 Hz
  //TPM1->MOD = 3276;
  
  // Set Modulo Value 48000000 / (128*50) = 50 Hz (50% duty cycle)
  TPM1->MOD = 7500; 
  
  // Edge-Aligned PWM
  // Update SnC register: CMOD = 01, PS=111 (128)
  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
  TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
  
  // Enable PWM on TPM1 Channel 0 -> PTB0
  TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
  TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
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


void InitGPIO(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));

}
