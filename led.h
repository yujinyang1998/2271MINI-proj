#ifndef LED_H_
#define LED_H_

#define PTB0_Pin 0  //buzzer
#define PTB8_PIN 8
#define PTB9_PIN 9
#define PTB10_PIN 10
#define PTB11_PIN 11
#define PTE2_PIN 2
#define PTE3_PIN 3
#define PTE4_PIN 4
#define PTE5_PIN 5

#define PTD7_PIN 7
#define PTD6_PIN 6
#define PTE31_PIN 31
#define PTA17_PIN 17
#define PTA16_PIN 16
#define PTC17_PIN 17
#define PTC16_PIN 16
#define PTC13_PIN 13
#define PTC12_PIN 12
#define MASK(x) (1 << (x))

void InitLed(void);

void GreenLED(int);

void RedLED(int);
#endif
