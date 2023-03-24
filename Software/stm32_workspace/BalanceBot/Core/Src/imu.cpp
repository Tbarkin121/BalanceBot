/*
 * imu.cpp
 *
 *  Created on: Feb 24, 2023
 *      Author: Plutonium
 */

#include "imu.h"
#include <stdio.h>
#include <cstring>
#include "math.h"

void IMU::init()
{
	IMU::reset();
	c_filter.tau = 0.01;
	first_loop = true;
	dt_clip = 1.0;
	ACC_FS = 2;
	GYRO_FS = 250;


}


void IMU::complimentary_update()
{

	c_filter.prev_time = c_filter.curr_time;
	c_filter.curr_time = sample.ms_counter;
	c_filter.dt = (float)(c_filter.curr_time - c_filter.prev_time)/1000.0;
	// Protection
	if(c_filter.dt > dt_clip || first_loop)
	{
		c_filter.dt = 0;
		first_loop = false;
	}

	c_filter.alpha = c_filter.tau/(c_filter.tau + c_filter.dt);
	// Low Pass Accel
	c_filter.acc_ang = atan2f((float)sample.acc_x, (float)sample.acc_z) * rad2deg;
	c_filter.acc_ang_lp =(1-c_filter.alpha)*c_filter.acc_ang + (c_filter.alpha)*c_filter.acc_ang_lp;
	// High Pass Gyro
	c_filter.gyro_ang_prev = c_filter.gyro_ang;
	c_filter.gyro_ang = c_filter.ang_est + sample.gyro_y*c_filter.dt/1000; // Divide by 1000 because gyro units in mili-dps
	c_filter.gyro_ang_hp = (1-c_filter.alpha)*c_filter.gyro_ang_hp + (1-c_filter.alpha)*(c_filter.gyro_ang - c_filter.gyro_ang_prev);

	c_filter.ang_est = (1-c_filter.alpha)*(c_filter.ang_est + c_filter.gyro_ang_hp * c_filter.dt) + (c_filter.alpha * c_filter.acc_ang_lp);
}

void IMU::reset()
{
	memset( &sample, 0, sizeof( sample ) );
    memset( &c_filter, 0, sizeof( c_filter ) );
}

float IMU::get_angle()
{
	return c_filter.ang_est;
}
