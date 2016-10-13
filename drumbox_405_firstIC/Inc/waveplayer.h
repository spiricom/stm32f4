/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/waveplayer.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for waveplayer.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */   
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEPLAYER_H
#define __WAVEPLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;



/* Exported constants --------------------------------------------------------*/
#define SAMPLE_RATE 48000.f
#define INV_SAMPLE_RATE 1.f/SAMPLE_RATE 
#define SAMPLE_RATE_MS (SAMPLE_RATE / 1000.f)
#define INV_SR_MS 1.f/SAMPLE_RATE_MS
#define SAMPLE_RATE_DIV_PARAMS 48000.f / 3
#define SAMPLE_RATE_DIV_PARAMS_MS (SAMPLE_RATE_DIV_PARAMS / 1000.f)
#define INV_SR_DIV_PARAMS_MS 1.f/SAMPLE_RATE_DIV_PARAMS_MS

#define VERY_SMALL_FLOAT 1.0e-38f
#define FREQ_KNOB_AVG_SIZE 1
#define INV_FREQ_KNOB_AVG_SIZE 1.f/FREQ_KNOB_AVG_SIZE
#define CROSSFADE_MAX 16
#define INV_CROSSFADE_MAX 1.f/CROSSFADE_MAX

#define DELAY_BUF_LENGTH 16384
#define MIN_FREQ (1.f / (DELAY_BUF_LENGTH / SAMPLE_RATE))
#define MAX_FREQ SAMPLE_RATE/2.0f

#define I2S_TIMEOUT 250

#define TWO_TO_8 256.f
#define INV_TWO_TO_8 1.f/TWO_TO_8
#define TWO_TO_5 32.f
#define INV_TWO_TO_5 1.0f/TWO_TO_5
#define TWO_TO_12 4096.f
#define INV_TWO_TO_12 1.f/TWO_TO_12
#define TWO_TO_15 32768.f
#define TWO_TO_16 65536.f
#define INV_TWO_TO_15 1.f/TWO_TO_15

#define ADC_BUFFER_LEN 32
#define ADC_INV_BUF_LEN 1/ADC_BUFFER_LEN
#define ENV_THRESHOLD -10.f

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void  audioInit(void);
void  changePhaseInc(float freq);

void DMA1_TransferCpltCallback(DMA_HandleTypeDef *hdma);
void DMA1_HalfTransferCpltCallback(DMA_HandleTypeDef *hdma);
#endif /* __WAVEPLAYER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
