/*
 * dynamixel.c
 *
 *  Created on: Mar 12, 2023
 *      Author: Plutonium
 */

#include "dynamixel.h"
#include "usart.h"

uint8_t dyna_msg[32];
unsigned short crc_value = 0;

// Instruction to write data to the device
void write()
{

}
//Instruction that registers the Instruction Packet to a standby status; Packet is later executed throught the Action command
void sync_write(int16_t current1, int16_t current2)
{

	dyna_msg[4] = 0xFE;
	dyna_msg[5] = 0x0D; //Length Low
	dyna_msg[6] = 0x00; //Length High
	dyna_msg[7] = 0x83; //Instruction (0x83 = SyncWrite)

	//Goal Torque Address : 0x0066
	dyna_msg[8] = 0x66;
	dyna_msg[9] = 0x00;
	// 2 Bytes Write
	dyna_msg[10] = 0x02;
	dyna_msg[11] = 0x00;

	// Motor 1 ID and Data
	dyna_msg[12] = 10;
	dyna_msg[13] = current1;
	dyna_msg[14] = current1>>8;
	// Motor 2 ID and Data
	dyna_msg[15] = 11;
	dyna_msg[16] = current2;
	dyna_msg[17] = current2>>8;

	//CRC
	crc_value = update_crc(0, dyna_msg, 18);
	dyna_msg[18] = crc_value; //CRC Low
	dyna_msg[19] = crc_value>>8; //CRC High
	HAL_UART_Transmit(&huart5, dyna_msg, 20, 1000);
}
// Instruction that executes the Packet that was registered beforehand using Reg Write


void dyna_init(uint8_t motor_id)
{
	// Setting Up Standard Header
	dyna_msg[0] = 0xFF; //Header 1
	dyna_msg[1] = 0xFF; //Header 2
	dyna_msg[2] = 0xFD; //Header 3
	dyna_msg[3] = 0x00; //Reserved


	disable_torque(motor_id);
	HAL_Delay(250);

	enable_current_mode(motor_id);
	HAL_Delay(250);

	enable_torque(motor_id);
	HAL_Delay(250);
}

void disable_torque(uint8_t motor_id)
{
	dyna_msg[4] = motor_id;
	dyna_msg[5] = 0x06; //Length Low
	dyna_msg[6] = 0x00; //Length High
	dyna_msg[7] = 0x03; //Instruction (0x03 = Write)
	dyna_msg[8] = 64; //Torque Enable Reg
	dyna_msg[9] = 0x00; //Torque Enable Reg
	dyna_msg[10] = 0;
	crc_value = update_crc(0, dyna_msg, 11);
	dyna_msg[11] = crc_value; //CRC Low
	dyna_msg[12] = crc_value>>8; //CRC High
	HAL_UART_Transmit(&huart5, dyna_msg, 13, 1000);

}

void enable_torque(uint8_t motor_id)
{
	dyna_msg[4] = motor_id;
	dyna_msg[5] = 0x06; //Length Low
	dyna_msg[6] = 0x00; //Length High
	dyna_msg[7] = 0x03; //Instruction (0x03 = Write)
	dyna_msg[8] = 64; //Torque Enable Reg
	dyna_msg[9] = 0x00; //Torque Enable Reg
	dyna_msg[10] = 1;
	crc_value = update_crc(0, dyna_msg, 11);
	dyna_msg[11] = crc_value; //CRC Low
	dyna_msg[12] = crc_value>>8; //CRC High
	HAL_UART_Transmit(&huart5, dyna_msg, 13, 1000);

}

void enable_current_mode(uint8_t motor_id)
{
	dyna_msg[4] = motor_id;
	dyna_msg[5] = 0x06; //Length Low
	dyna_msg[6] = 0x00; //Length High
	dyna_msg[7] = 0x03; //Instruction (0x03 = Write)
	dyna_msg[8] = 11; //Control Mode Reg
	dyna_msg[9] = 0x00; //Control Mode Reg
	dyna_msg[10] = 0;
	crc_value = update_crc(0, dyna_msg, 11);
	dyna_msg[11] = crc_value; //CRC Low
	dyna_msg[12] = crc_value>>8; //CRC High
	HAL_UART_Transmit(&huart5, dyna_msg, 13, 1000);
}

void write_torque(uint8_t motor_id, int16_t target_torque)
{
	dyna_msg[4] = motor_id;
	dyna_msg[5] = 0x07; //Length Low
	dyna_msg[6] = 0x00; //Length High
	dyna_msg[7] = 0x03; //Instruction (0x03 = Write)
	dyna_msg[8] = 0x66; //Params 1 (Address 0x0066 Low)
	dyna_msg[9] = 0x00; //Params 2 (Address 0x0066 High)

	int16_t current = target_torque; //mA
	dyna_msg[10] = current;
	dyna_msg[11] = current>>8;
	//  dyna_msg[8] = 0x00; //CRC Low
	//  dyna_msg[9] = 0x00; //CRC High

	crc_value = update_crc(0, dyna_msg, 12);
	dyna_msg[12] = crc_value; //CRC Low
	dyna_msg[13] = crc_value>>8; //CRC High
	HAL_UART_Transmit(&huart5, dyna_msg, 14, 1000);
}



unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
