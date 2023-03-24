/*
 * dynamixel.h
 *
 *  Created on: Mar 12, 2023
 *      Author: Plutonium
 */





#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "stdio.h"

void dyna_init(uint8_t motor_id);
void disable_torque(uint8_t motor_id);
void enable_torque(uint8_t motor_id);
void enable_current_mode(uint8_t motor_id);
void write_torque(uint8_t motor_id, int16_t target_torque);

// Instruction to write data to the device
void write();
//Instruction that registers the Instruction Packet to a standby status; Packet is later executed throught the Action command
void sync_write(int16_t current1, int16_t current2);

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);


#endif /* INC_TEST_H_ */
