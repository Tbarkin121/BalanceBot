/*
 * motor.h
 *
 *  Created on: Sep 23, 2022
 *      Author: Plutonium
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>


typedef struct
{
  uint32_t ms_counter;
  int32_t acc_x;
  int32_t acc_y;
  int32_t acc_z;
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
} IMU_Measurement;


typedef struct
{
  float tau;
  float alpha;
  float acc_ang;
  float acc_ang_lp;
  float gyro_ang;
  float gyro_ang_prev;
  float gyro_ang_hp;
  float ang_est;
  uint32_t curr_time;
  uint32_t prev_time;
  float dt;
} Complimentary_Filter;

class IMU {
  private:
	Complimentary_Filter c_filter;
	bool first_loop;
	float dt_clip;
	uint8_t ACC_FS;
	uint8_t GYRO_FS;
    float rad2deg = 57.2958;

  public:
	IMU_Measurement sample; 				// The current imu measurement
    void init();
    void complimentary_update();
    void reset();
    float get_angle();
};



#endif /* INC_IMU_H_ */
