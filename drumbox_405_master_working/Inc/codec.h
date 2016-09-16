
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

#define CODEC_I2C_ADDRESS (0x1a << 1) // 7-bit address goes one bit over to the left to make room for R/W bit

#ifndef ADCHPD
  #define ADCHPD 0
#elif (ADCHPD == 0)||(ADCHPD == 1)
#else
  #error ADCHPD value not defined
#endif

#ifndef ADCS
  #define ADCS 2
#elif (ADCS >=0)&&(ADCS <= 2)
#else
  #error ADCS value not defined
#endif

#ifndef HYST
  #define HYST 32
#elif (HYST >= 0)&&(HYST <= 255)
#else
  #error HYST value not defined
#endif

#ifndef LINVOL
  #define LINVOL 0x17
#elif (LINVOL >= 0) && (LINVOL <= 0x1f)
#else
  #error LINVOL value not defined
#endif

#ifndef RINVOL
  #define RINVOL 0x17
#elif (RINVOL >= 0) && (RINVOL <= 0x1f)
#else
  #error RINVOL value not defined
#endif

#ifndef LHPVOL
  #define LHPVOL 121
#elif (LHPVOL == 0) || ((LHPVOL >= 0x30) && (LHPVOL <= 0x7f))
#else
  #error LHPVOL value not defined
#endif

#ifndef RHPVOL
  #define RHPVOL 121
#elif (RHPVOL == 0) || ((RHPVOL >= 0x30) && (RHPVOL <= 0x7f))
#else
  #error RHPVOL value not defined
#endif

#ifndef MICBOOST
  #define MICBOOST 0
#elif (MICBOOST == 0)||(MICBOOST == 1)
#else
  #error MICBOOST value not defined
#endif

	// 1 = muted
#ifndef MUTEMIC
  #define MUTEMIC 1
#elif (MUTEMIC == 0)||(MUTEMIC == 1)
#else
  #error MUTEMIC value not defined
#endif

	// 0 = line inputs, 1 = mic in
#ifndef INSEL
  #define INSEL 0
#elif (INSEL == 0)||(INSEL == 1)
#else
  #error INSEL value not defined
#endif

#ifndef BYPASS
  #define BYPASS 0 //setting this to 1 passes the line input straight to the line output
#elif (BYPASS == 0)||(BYPASS == 1)
#else
  #error BYPASS value not defined
#endif

#ifndef DACSEL
  #define DACSEL 1
#elif (DACSEL == 0)||(DACSEL == 1)
#else
  #error DACSEL value not defined
#endif

#ifndef SIDETONE
  #define SIDETONE 0
#elif (SIDETONE == 0)||(SIDETONE == 1)
#else
  #error SIDETONE value not defined
#endif

#ifndef SIDEATT
  #define SIDEATT 0
#elif (SIDEATT >= 0)&&(SIDEATT <= 3)
#else
  #error SIDEATT value not defined
#endif

uint16_t i2cDataSize = 2;
uint8_t myI2cData[2] = {0,0};
uint32_t I2Ctimeout = 2000;

void AudioCodec_init(void) {

	//now to send all the necessary messages to the codec (pack them in myI2cData)
	//reset the DAC
	myI2cData[0] = 0x1e;
  myI2cData[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
	
	//power reduction register - turn power on
	myI2cData[0] = 0x0c;
  myI2cData[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
	//digital data format - I2S 
	myI2cData[0] = 0x0e;
  myI2cData[1] = 0x02;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);

  //left in setup register
	myI2cData[0] = 0x00;
  myI2cData[1] = LINVOL;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
	
  //right in setup register
	myI2cData[0] = 0x02;
  myI2cData[1] = RINVOL;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
  //left headphone out register
	myI2cData[0] = 0x04;
  myI2cData[1] = LHPVOL;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
  //right headphone out register
	myI2cData[0] = 0x06;
  myI2cData[1] = RHPVOL;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
  //digital audio path configuration
	myI2cData[0] = 0x0a;
  myI2cData[1] = ADCHPD;
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
  //analog audio path configuration
	myI2cData[0] = 0x08;
  myI2cData[1] = ((uint8_t)((SIDEATT << 6)|(SIDETONE << 5)|(DACSEL << 4)|(BYPASS << 3)|(INSEL << 2)|(MUTEMIC << 1)|(MICBOOST << 0)));
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
	
	//clock configuration
	myI2cData[0] = 0x10;
  myI2cData[1] = 0x00; 
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);
 
	//codec enable
	myI2cData[0] = 0x12;
  myI2cData[1] = 0x01; 
	HAL_I2C_Master_Transmit(&hi2c1, CODEC_I2C_ADDRESS, myI2cData, i2cDataSize, I2Ctimeout);

}
