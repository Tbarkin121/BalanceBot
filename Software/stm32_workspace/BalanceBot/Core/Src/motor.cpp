/*
 * motor.cpp
 *
 *  Created on: Sep 23, 2022
 *      Author: Plutonium
 */

#include "motor.h"
#include <stdio.h>

void Motor::fun()
{
	printf("motor test function\n");
}

void Motor::PID_Set_Gains(float kd, float ki, float kp)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
}
void Motor::PID_Init(float kp, float ki, float kd, float out_thresh)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
	A0 = Kp + Ki + Kd;
	A1 = (-Kp ) - (2 * Kd );
	A2 = Kd;
	pid_state[0] = 0.0;
	pid_state[1] = 0.0;
	pid_state[2] = 0.0;
	output = 0.0;
	output_threshold = out_thresh;
}
float Motor::PID_Controller(float target)
{
	float error = target - state.Vel;
//	y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2];

	output = pid_state[2] + A0 * error + A1 * pid_state[0] + A2 * pid_state[1];
	float part1 = pid_state[2];
	float part2 = A0 * error;
	float part3 = A1 * pid_state[0];
	float part4 = A2 * pid_state[1];
	float total = part1 + part2 + part3 + part4;
//	output = total*output_scale;
//	output = output*output_scale;
	// Thresholding
	if(output > output_threshold)
		output = output_threshold;
	if(output < - output_threshold)
		output = -output_threshold;

	pid_state[1] = pid_state[0];
	pid_state[0] = error;
	pid_state[2] = output;

	return output;
}
