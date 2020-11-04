#ifndef MOTORS_H_
#define MOTORS_H_

#define PTD0_Pin 0 	//Left forward
#define PTD1_Pin 1	//Left reverse
#define PTD2_Pin 2	//Right forward
#define PTD3_Pin 3	//Right reverse
#define ROBOT_FORWARD 17
#define ROBOT_BACKWARD 18
#define ROBOT_LEFT 20
#define ROBOT_RIGHT 24
#define ROBOT_STOP 0
#define FORWARD_LEFT 21
#define FORWARD_RIGHT 25
#define REVERSE_LEFT 22
#define REVERSE_RIGHT 26
#define FAST 100
#define SLOW 75

#define FREQ_MOD(x) (375000/x)17

int PWM(int);

void move(int);

void InitMotor(void);
#endif
