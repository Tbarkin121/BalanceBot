/*
 * motor.h
 *
 *  Created on: Sep 23, 2022
 *      Author: Plutonium
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

typedef struct
{
  int16_t Ang;    	 /*!< Specifies the mechanical angle of the motor*/

  int16_t Vel;    	 /*!< Specifies the rotational rate of the motor*/

  int16_t Torq;     /*!< Specifies the torque current of the motor*/

  int8_t Temp;     /*!< Specifies the temperature of the motor*/

} MotorState;


class Motor {
  private:
	float A0; 				// The derived gain, A0 = Kp + Ki + Kd .
	float A1; 				// The derived gain, A1 = -Kp - 2Kd.
	float A2; 				// The derived gain, A2 = Kd .
	float pid_state[3]; 		// The state array of length 3.
	float Kd; 				// The derivative gain.
	float Ki; 				// The integral gain.
	float Kp; 				// The proportional gain.
	float output;
	float output_scale;     // Because 1000 units of error shouldn't equal 1000 Amps
	float output_threshold; // I want to start with some safety limits so I don't break anything

  public:
	MotorState state;
	void fun();
	void PID_Init(float kp, float ki, float kd, float out_thresh);
	void PID_Set_Gains(float kp, float ki, float kd);
	float PID_Controller(float error);
};



#endif /* INC_MOTOR_H_ */
