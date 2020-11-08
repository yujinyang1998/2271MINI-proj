#include "MKL25Z4.h"                    // Device header
#include "motor.h"

void move(int dir) {
	switch(dir){
	case (ROBOT_FORWARD):
			TPM0_C0V = PWM(FAST);
			TPM0_C1V = 0;
			TPM0_C2V = PWM(FAST);
			TPM0_C3V = 0;
			break;
	case(ROBOT_BACKWARD):
			TPM0_C0V = 0;
			TPM0_C1V = PWM(FAST);
			TPM0_C2V = 0;
			TPM0_C3V = PWM(FAST);
	  	break;
	case(ROBOT_RIGHT):
			TPM0_C0V = 0;
			TPM0_C1V = PWM(MEDIUM);
			TPM0_C2V = PWM(MEDIUM);
			TPM0_C3V = 0;
		  break;
	case(ROBOT_LEFT):
			TPM0_C0V = PWM(MEDIUM);
			TPM0_C1V = 0;
			TPM0_C2V = 0;
			TPM0_C3V = PWM(MEDIUM);
			break;
	case(FORWARD_RIGHT):
			TPM0_C0V = PWM(SLOW);
			TPM0_C1V = 0;
			TPM0_C2V = PWM(FAST);
			TPM0_C3V = 0;
			break;
	case(FORWARD_LEFT):
			TPM0_C0V = PWM(FAST);
			TPM0_C1V = 0;
			TPM0_C2V = PWM(SLOW);
			TPM0_C3V = 0;
			break;
	case(REVERSE_RIGHT):
			TPM0_C0V = 0;
			TPM0_C1V = PWM(SLOW);
			TPM0_C2V = 0;
			TPM0_C3V = PWM(FAST);
	  	break;
	case(REVERSE_LEFT):
			TPM0_C0V = 0;
			TPM0_C1V = PWM(FAST);
			TPM0_C2V = 0;
			TPM0_C3V = PWM(SLOW);
	  	break;
	default:
			TPM0_C0V = 0;
			TPM0_C1V = 0;
			TPM0_C2V = 0;
			TPM0_C3V = 0;
			break;
	}
}

int PWM(int duty_cycle){
	return ((float) duty_cycle / 100) * (256);
}

void InitMotor(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Configure MUX setting
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH0
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK;		
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH1
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH2
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);			//TPM0_CH3

	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	TPM0->MOD = 255;

	// Edge-Aligned PWM 
	// CMOD = 01 (count up), PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

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
