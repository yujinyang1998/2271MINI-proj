#ifndef MOTORS_H_
#define MOTORS_H_

#define PTD0_Pin 0 	//Left forward
#define PTD1_Pin 1	//Left reverse
#define PTD2_Pin 2	//Right forward
#define PTD3_Pin 3	//Right reverse
#define ROBOT_FORWARD 1
#define ROBOT_BACKWARD 2
#define ROBOT_LEFT 3
#define ROBOT_RIGHT 4
#define ROBOT_STOP 0
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define REVERSE_LEFT 7
#define REVERSE_RIGHT 8
#define FAST 100
#define SLOW 75

#define FREQ_MOD(x) (375000/x)

int PWM(int);

void move(int);

void InitMotor();
#endif