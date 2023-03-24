/**
******************************************************************************
* @file    usbd_audio_in_if_template.c
* @author  Central Labs
* @version V2.4.2 MOD
* @date    01-Sep-2016
* @brief   USB Device Audio interface template.
******************************************************************************
@attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if_template.h"

#ifdef USE_STM32L4XX_NUCLEO
extern uint16_t PCM_Buffer[];
#else
extern uint16_t PDM_Buffer[];
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t TEMPLATE_Init(uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
static int8_t TEMPLATE_DeInit(uint32_t options);
static int8_t TEMPLATE_Record(void);
static int8_t TEMPLATE_VolumeCtl(int16_t Volume);
static int8_t TEMPLATE_MuteCtl(uint8_t cmd);
static int8_t TEMPLATE_Stop(void);
static int8_t TEMPLATE_Pause(void);
static int8_t TEMPLATE_Resume(void);
static int8_t TEMPLATE_CommandMgr(uint8_t cmd);

/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef hUSBDDevice;
USBD_AUDIO_ItfTypeDef USBD_Template_fops = {
  TEMPLATE_Init,
  TEMPLATE_DeInit,
  TEMPLATE_Record,
  TEMPLATE_VolumeCtl,
  TEMPLATE_MuteCtl,
  TEMPLATE_Stop,
  TEMPLATE_Pause,
  TEMPLATE_Resume,
  TEMPLATE_CommandMgr,
};


/* Private functions ---------------------------------------------------------*/
/* This table maps the audio device class setting in 1/256 dB to a
* linear 0-64 scaling used in pdm_filter.c. It is computed as
* 256*20*log10(index/64). */
const int16_t vol_table[65] =
{ 0x8000, 0xDBE0, 0xE1E6, 0xE56B, 0xE7EB, 0xE9DB, 0xEB70, 0xECC7,
0xEDF0, 0xEEF6, 0xEFE0, 0xF0B4, 0xF176, 0xF228, 0xF2CD, 0xF366,
0xF3F5, 0xF47C, 0xF4FB, 0xF574, 0xF5E6, 0xF652, 0xF6BA, 0xF71C,
0xF778, 0xF7D6, 0xF82D, 0xF881, 0xF8D2, 0xF920, 0xF96B, 0xF9B4,
0xF9FB, 0xFA3F, 0xFA82, 0xFAC2, 0xFB01, 0xFB3E, 0xFB79, 0xFBB3,
0xFBEB, 0xFC22, 0xFC57, 0xFC8C, 0xFCBF, 0xFCF1, 0xFD22, 0xFD51,
0xFD80, 0xFDAE, 0xFDDB, 0xFE07, 0xFE32, 0xFE5D, 0xFE86, 0xFEAF,
0xFED7, 0xFF00, 0xFF25, 0xFF4B, 0xFF70, 0xFF95, 0xFFB9, 0xFFD0,
0x0000 };


/**
* @brief  Initializes the TEMPLATE media low layer.
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @param  BitRes: desired bit resolution
* @param  ChnlNbr: number of channel to be configured
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_Init(uint32_t  AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{
  return AUDIO_OK;
}

/**
* @brief  De-Initializes the TEMPLATE media low layer.      
* @param  options: Reserved for future use
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_DeInit(uint32_t options)
{
  return AUDIO_OK;
}

/**
* @brief  Start audio recording engine
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_Record(void)
{
  return AUDIO_OK;
}

/**
* @brief  Controls TEMPLATE Volume.             
* @param  vol: Volume level
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_VolumeCtl(int16_t Volume)
{
  return AUDIO_OK;
}

/**
* @brief  Controls TEMPLATE Mute.              
* @param  cmd: Command opcode
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_MuteCtl(uint8_t cmd)
{
  return AUDIO_OK;
}


/**
* @brief  Stops audio acquisition
* @param  none
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_Stop(void)
{  
  return AUDIO_OK;
}

/**
* @brief  Pauses audio acquisition
* @param  none
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/

static int8_t TEMPLATE_Pause(void)
{
  return AUDIO_OK;
}


/**
* @brief  Resumes audio acquisition
* @param  none
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/
static int8_t TEMPLATE_Resume(void)
{  
  return AUDIO_OK;
}

/**
* @brief  Manages command from usb
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise
*/

static int8_t TEMPLATE_CommandMgr(uint8_t cmd)
{
  return AUDIO_OK;
}
/**
* @brief  Fills USB audio buffer with the right amount of data, depending on the
*			channel/frequency configuration
* @param  audioData: pointer to the PCM audio data
* @param  PCMSamples: number of PCM samples to be passed to USB engine
* @note Depending on the calling frequency, a coherent amount of samples must be passed to
*       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel,
*       you can pass 16 PCM samples if the function is called each millisecond,
*       32 samples if called every 2 milliseconds and so on.
*/
void Send_Audio_to_USB(int16_t * audioData, uint16_t PCMSamples){
  
  USBD_AUDIO_Data_Transfer(&hUSBDDevice, (int16_t *)audioData, PCMSamples);
}









/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
