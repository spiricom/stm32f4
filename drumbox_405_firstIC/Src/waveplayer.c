/**
This is the Snyderphonics DrumBox synthesis code. 
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "waveplayer.h"
#include "i2s.h"
#include "i2c.h"
#include "dma.h"
#include "codec.h"
#include "rng.h"
#include "stm32f4xx_hal_i2s_ex.h"
#include "math.h"
#include "spi.h"
#include "dma.h"

#include "main.h"
#include "waveplayer.h"
#include "wavetables.h"
#include "audiounits.h"
#include "utilities.h"

#define TEST_LED_ENABLED 0
#define USE_NEW_FEEDBACK 0

#define AUDIO_BUFFER_SIZE             32 //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)
#define NUM_SMOOTHED_PARAMS 3
#define NUM_KNOBS 6
#define NUM_ADC_VALUES 10
#define CAT_BUFFERSIZE 2
#define TIMEOUT 10
#define NUM_LEDS 8

#define IC_BUFFER_SIZE 4
#define HALF_IC_BUFFER_SIZE (IC_BUFFER_SIZE / 2)
#define IC_TIMEOUT 200

#define DELAY_BUFFER_LENGTH 16384

#define NUM_FB_DELAY_TABLES 8

/* Ping-Pong buffer used for audio play */
int16_t Send_Buffer[AUDIO_BUFFER_SIZE];
int16_t Receive_Buffer[AUDIO_BUFFER_SIZE];

uint8_t whichKnob;

float smoothedParams[NUM_SMOOTHED_PARAMS];
float delayBuffer1[DELAY_BUFFER_LENGTH];

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_i2s3_ext_rx;
extern I2S_HandleTypeDef hi2s3;
extern RNG_HandleTypeDef hrng; // noise number generator
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


// IC communication variables
uint16_t i2cDataSize2 = 2;
uint8_t myTouchpad[2];
uint32_t I2Ctimeout2 = 2000;
uint8_t ic_counter = 0;
uint8_t IC_tx[IC_BUFFER_SIZE];
uint8_t IC_rx[IC_BUFFER_SIZE];
uint8_t ICbufferTx[IC_BUFFER_SIZE*10];
uint8_t ICbufferRx[IC_BUFFER_SIZE*10];

uint16_t ADC_values[NUM_ADC_VALUES];

// Sine 
tCycle sin1; 

// Noise 
tNoise noise1;

// Ramp
tRamp rampFeedback;
tRamp rampSineFreq;
tRamp rampDelayFreq;

// Highpass
tHighpass highpass1;
tHighpass highpass2; 

// Envelope follower
tEnvelopeFollower envFollowNoise;
tEnvelopeFollower envFollowSine;

// Delay 
tDelay delay1;

// Gain
float masterGain = 0.3f;
float sineGain = 0.8f;
float noiseGain = 0.8f; 
float ksGain = 0.7f;

float feedbacksamp = 0.f;

float newFreq = 0.0f;
float newDelay = 0.0f;
float newFeedback = 0.0f;


float m_input1 = 0.f;
float	m_output0 = 0.f;

#define FEEDBACK_LOOKUP_SIZE 5
#define DELAY_LOOKUP_SIZE 4
float FeedbackLookup[FEEDBACK_LOOKUP_SIZE] = { 0.0f, 0.8f, .999f, 1.0f, 1.001f };
//float DelayLookup[DELAY_LOOKUP_SIZE] = { 16000.f, 1850.f, 180.f, 40.f };
float DelayLookup[DELAY_LOOKUP_SIZE] = { 50.f, 180.f, 1400.f, 16300.f };

float feedbackDelayPeriod[NUM_FB_DELAY_TABLES];
const float *feedbackDelayTable[NUM_FB_DELAY_TABLES] = { FB1, FB2, FB3, FB4, FB5, FB6, FB7, FB8 };


uint8_t ADC_out[2];
uint8_t ADC_in[2];


uint8_t CAT_LED[CAT_BUFFERSIZE];


#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

/* Private function prototypes -----------------------------------------------*/
// PROTOTYPES

// IC communication functions
void start_Other_IC_Communication(void);
void sendBytesToOtherIC(uint8_t startingByte, uint8_t data);
uint8_t readBytesFromOtherIC(uint8_t startingByte);
int16_t readAndWriteOtherChip(int16_t, uint16_t);
void WRITE_FLOAT_AS_BYTES(uint8_t byteNum, float data);
float READ_BYTES_AS_FLOAT(uint8_t byteNum);
void WRITE_FLOAT16_AS_BYTES(uint8_t byteNum, float data);
float READ_BYTES_AS_FLOAT16(uint8_t byteNum);
void WRITE_INT16_AS_BYTES(uint8_t byteNum, int16_t data);
int16_t READ_BYTES_AS_INT16(uint8_t byteNum);

// Audio functions
float ksTick(float);
void readExternalADC(void);
float audioProcess(float);
float interpolateFeedback(uint16_t idx);
float interpolateDelayControl(float raw_data);
void audioTick(uint16_t buffer_offset);


void Write7SegWave(uint8_t value);
void getXY_Touchpad(void);

/* Private functions ---------------------------------------------------------*/
void Error_Handler(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (GPIO_PinState) 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (GPIO_PinState) 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (GPIO_PinState) 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState) 0);
  while(1)
  {
		;
  }
}

// Returns random floating point value [0.0,1.0)
float randomNumber(void) {
	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (((float)(rand >> 16))- 32768.f) * INV_TWO_TO_15;
	return num;
}

uint16_t mess = 0;
	static HAL_DMA_StateTypeDef spi1tx;
	static HAL_DMA_StateTypeDef spi1rx;

void setXYLED(uint8_t led1, uint8_t led2) {
		
	if (led1 < NUM_LEDS) {
		CAT_LED[0] = (1 << led1);
	} else {
		CAT_LED[0] = 0;
	}
	if (led2 < NUM_LEDS) {
		CAT_LED[1] = (1 << led2);
	} else {
		CAT_LED[1] = 0;
	}
		
		HAL_SPI_Transmit(&hspi2, (uint8_t *)CAT_LED, CAT_BUFFERSIZE, TIMEOUT);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // CAT4016 Latch pin goes high to latch data
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CAT4016 Latch pin goes back low to reset it
}

void setTrigOutLED(uint8_t set) {
	if (set) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}
	

void audioInit(void)
{ 

	int16_t dummy1 = 0;
	uint16_t dummy2 = 0;
	
	float fbdp = 0.0002604165; //min feedback delay period
	for (int i = 0; i < NUM_FB_DELAY_TABLES; i++) {
		feedbackDelayPeriod[i] = fbdp;
		fbdp *= 2.0f;
	}

	// Initialize cycle.
	// initialize audio 
	tCycleInit(&sin1, SAMPLE_RATE, sinewave, SINE_TABLE_SIZE);
	tNoiseInit(&noise1, SAMPLE_RATE, &randomNumber, NoiseTypePink);
	tRampInit(&rampFeedback, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampSineFreq, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampDelayFreq, SAMPLE_RATE, 10.0f, 1);
	tHighpassInit(&highpass1, SAMPLE_RATE, 20.0f);
	tHighpassInit(&highpass2, SAMPLE_RATE, 45.0f);
	tEnvelopeFollowerInit(&envFollowNoise, 0.05, 0.0f);
	tEnvelopeFollowerInit(&envFollowSine, 0.05, 0.0f);
	tDelayInit(&delay1,delayBuffer1);
	

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high to make sure it's not selected
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
	HAL_Delay(100);

#if TEST_LED_ENABLED
	int x,y;
	int timer = 0;
	int toggle = 0;
	while(1) {
		
		setXYLED(x,y);
		
		
		if (timer++ > 100000) {
			toggle = !toggle;
			setTrigOutLED(toggle);
			x += 2; y += 3;
			timer = 0;
		}
		if (x >= 8) x = 0;
		if (y >= 8) y = 0;
			
	}
#endif
	//now to send all the necessary messages to the codec
	AudioCodec_init();
	HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	
	//MX_SPI1_Init();
	//firstXY_Touchpad();
	//start_Other_IC_Communication();
	
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
  
	readAndWriteOtherChip(dummy1, dummy2);
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
/*
  while(1)
	{
		//if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
		//{
			//getXY_Touchpad();
			//Write7SegWave((myTouchpad[0] / 4));
		  spi1tx = HAL_DMA_GetState(&hdma_spi1_tx);
		  mess = spi1tx;
		  spi1rx = HAL_DMA_GetState(&hdma_spi1_rx);
		  mess = spi1rx;
		
		
		//}
	}
	*/
	/*
	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
		{
			getXY_Touchpad();
			//Write7SegWave((myTouchpad[0] / 4));
			
		}
	}
	*/
	//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
}

void audioTick(uint16_t buffer_offset)
{
			uint16_t i = 0;
			int16_t current_sample = 0;    
	
			for (i = 0; i < (HALF_BUFFER_SIZE); i++)
			{
				if ((i & 1) == 0)
				{
					current_sample = (int16_t)(audioProcess((float) (Receive_Buffer[HALF_BUFFER_SIZE + i] * INV_TWO_TO_15)) * TWO_TO_15);
				}
				Send_Buffer[buffer_offset + i] = current_sample;
			}
			readExternalADC();
}




float audioProcess(float audioIn) {
	
	envFollowNoise.decayCoeff(&envFollowNoise,adc1[ADC_values[ControlParameterNoiseDecay]]);
  envFollowSine.decayCoeff(&envFollowSine,adc1[ADC_values[ControlParameterSineDecay]]); 	
	
	//set frequency of sine and delay
	sin1.freq(&sin1, mtof1[(uint16_t)(smoothedParams[ControlParameterFrequency] * 4096.0f)]);
	
	//setDelay(smoothedParams[ControlParameterDelay]);
	delay1.setDelay(&delay1, smoothedParams[ControlParameterDelay]);
	
	
	float newAttackThresh = ((float)ADC_values[ControlParameterThreshold]) * INV_TWO_TO_12 * 0.2f;
	envFollowNoise.attackThresh(&envFollowNoise,newAttackThresh);
  envFollowSine.attackThresh(&envFollowSine,newAttackThresh); 	
	
	float sample = 0.8f * audioIn;
	if (ADC_values[ControlParameterFeedback] > 5) 
		sample += (ksGain * ksTick(audioIn));
	if (ADC_values[ControlParameterSineDecay] > 5) 
		sample += (sineGain * sin1.tick(&sin1) * envFollowSine.tick(&envFollowSine,audioIn));
	if (ADC_values[ControlParameterNoiseDecay] > 5) 
		sample += (noiseGain * noise1.tick(&noise1) * envFollowNoise.tick(&envFollowNoise,audioIn));
	
	sample *= masterGain;
  sample = highpass1.tick(&highpass1, shaper1[(uint16_t)((sample+1.0f)*0.5f * TWO_TO_15)]);

	//update Parameters
	smoothedParams[ControlParameterFeedback] = rampFeedback.tick(&rampFeedback);
	smoothedParams[ControlParameterFrequency] = rampSineFreq.tick(&rampSineFreq);
	smoothedParams[ControlParameterDelay] = rampDelayFreq.tick(&rampDelayFreq);

  return sample;
}
float clip(float min, float val, float max) {
	
	if (val < min) {
		return min;
	} else if (val > max) {
		return max;
	} else {
		return val;
	}
}

float ksTick(float noise_in)
{
		float temp_sample;
	  
		temp_sample = noise_in + (feedbacksamp * smoothedParams[ControlParameterFeedback]);
		
		//feedbacksamp = delayTick(temp_sample);
		feedbacksamp = delay1.tick(&delay1, temp_sample);
				
	  //simple one-zero lowpass filter (moving average)
		m_output0 = 0.5f * m_input1 + 0.5f * feedbacksamp;
		m_input1 = feedbacksamp;
		feedbacksamp = clip(-1.0f,highpass2.tick(&highpass2,m_output0),1.0f);
		return shaper1[(uint16_t)(((feedbacksamp + 1.0f) * 0.5f) * (TWO_TO_16-1))];
}


void readExternalADC(void)
{
	uint8_t knobnum;
	uint16_t knobval12bit;
	uint16_t knobsendbyte;
	uint8_t ADC_out[2];

	//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
	knobsendbyte = 6208 + (whichKnob << 7);
	ADC_out[0] = knobsendbyte >> 8;
	ADC_out[1] = knobsendbyte & 255;
	
	// write to ADC chip (asking for data on next channel)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ADC CS pin goes low to initiate conversion
	//for(int j = 0; j < 5000; j++);

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)ADC_out, (uint8_t *)ADC_in, ADC_BUFFERSIZE, TIMEOUT);
	//for(int j = 0; j < 5000; j++);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high

	knobnum = ADC_in[0] >> 4;
	knobval12bit = (ADC_in[1] | ((ADC_in[0] & 15) << 8));
	ADC_values[knobnum] = knobval12bit;
	
	if (knobnum == ControlParameterFeedback){
		
		newFeedback = interpolateFeedback(ADC_values[ControlParameterFeedback]);
		rampFeedback.setDest(&rampFeedback,newFeedback);
		
	} else if (knobnum == ControlParameterDelay) {
		
		newDelay = interpolateDelayControl((4096 - ADC_values[ControlParameterDelay]));
		rampDelayFreq.setDest(&rampDelayFreq,newDelay);
		
	} else if (knobnum == ControlParameterFrequency) {
		
		newFreq =  (ADC_values[ControlParameterFrequency] * INV_TWO_TO_12) /* * (ADC_values[0] * INV_TWO_TO_12)*/; 
		rampSineFreq.setDest(&rampSineFreq,newFreq);
		
	}

	if (++whichKnob >= NUM_KNOBS)
	{
		whichKnob = 0;
	}
}

float interpolateDelayControl(float raw_data)
{
	float scaled_raw = raw_data * INV_TWO_TO_12;
	if (scaled_raw < 0.2f)
	{
		return (DelayLookup[0] + ((DelayLookup[1] - DelayLookup[0]) * (scaled_raw * 5.f)));
	}
	else if (scaled_raw < 0.6f)
	{
		return (DelayLookup[1] + ((DelayLookup[2] - DelayLookup[1]) * ((scaled_raw - 0.2f) * 2.5f)));
	}
	else
	{
		return (DelayLookup[2] + ((DelayLookup[3] - DelayLookup[2]) * ((scaled_raw - 0.6f) * 2.5f)));
	}	
}
float out;
uint16_t index1;
float delPer;
float w1,w2,range;
float interpolateFeedback(uint16_t idx) {
	
	index1 = idx;
	float delayPeriod = (smoothedParams[ControlParameterDelay] * INV_SAMPLE_RATE);
	delPer = delayPeriod;
	
	
	if (delayPeriod < feedbackDelayPeriod[0]) {
			out = FB1[idx];
	} else if (delayPeriod < feedbackDelayPeriod[1]) {
			// interp between fb[i] and fb[i-1]
			range = feedbackDelayPeriod[1] - feedbackDelayPeriod[0]; 
			w1 = (feedbackDelayPeriod[1] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB1[idx] * w1) + (FB2[idx] * w2); 	
	} else if (delayPeriod < feedbackDelayPeriod[2]) {
		
			range = feedbackDelayPeriod[2] - feedbackDelayPeriod[1]; 
			w1 = (feedbackDelayPeriod[2] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB2[idx] * w1) + (FB3[idx] * w2); 	
	} else if (delayPeriod < feedbackDelayPeriod[3]) {
		
			range = feedbackDelayPeriod[3] - feedbackDelayPeriod[2]; 
			w1 = (feedbackDelayPeriod[3] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB3[idx] * w1) + (FB4[idx] * w2);
	} else if (delayPeriod < feedbackDelayPeriod[4]) {
		
			range = feedbackDelayPeriod[4] - feedbackDelayPeriod[3]; 
			w1 = (feedbackDelayPeriod[4] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB4[idx] * w1) + (FB5[idx] * w2);
	} else if (delayPeriod < feedbackDelayPeriod[5]) {
		
			range = feedbackDelayPeriod[5] - feedbackDelayPeriod[4]; 
			w1 = (feedbackDelayPeriod[5] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB5[idx] * w1) + (FB6[idx] * w2);
	} else if (delayPeriod < feedbackDelayPeriod[6]) {
		
			range = feedbackDelayPeriod[6] - feedbackDelayPeriod[5]; 
			w1 = (feedbackDelayPeriod[6] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB6[idx] * w1) + (FB7[idx] * w2);
	} else if (delayPeriod < feedbackDelayPeriod[7]) {
		
			range = feedbackDelayPeriod[7] - feedbackDelayPeriod[6]; 
			w1 = (feedbackDelayPeriod[7] - delayPeriod)/range;
			w2 = 1.0f - w1; 

			out = (FB7[idx] * w1) + (FB8[idx] * w2);
	}else  {
			out = FB8[idx] ;
	} 
	
	return out;
	

/*
	float scaled_raw = idx * INV_TWO_TO_12;
	if (scaled_raw < 0.2f)
	{
		return (FeedbackLookup[0] + ((FeedbackLookup[1] - FeedbackLookup[0]) * ((scaled_raw) * 5.0f)));
	}		
	else if (scaled_raw < 0.6f)
	{
		return (FeedbackLookup[1] + ((FeedbackLookup[2] - FeedbackLookup[1]) * ((scaled_raw - 0.2f) * 2.5f)));
	}		
	if (scaled_raw < .95f)
	{
		return (FeedbackLookup[2] + ((FeedbackLookup[3] - FeedbackLookup[2]) * ((scaled_raw - 0.6f) * 2.857142857142f)));
	}
	else
	{
		return (FeedbackLookup[3] + ((FeedbackLookup[4] - FeedbackLookup[3]) * ((scaled_raw - 0.95f) * 20.0f )));
	}
*/

}

void Write7SegWave(uint8_t value)
{
	uint8_t Digits[2];
	Digits[0] = value % 10;
	Digits[1] = value / 10;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (GPIO_PinState)(Digits[0] & 1));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)(Digits[0] & 2));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)(Digits[0] & 4));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)(Digits[0] & 8));
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (GPIO_PinState)(Digits[1] & 1));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (GPIO_PinState)(Digits[1] & 2));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (GPIO_PinState)(Digits[1] & 4));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState)(Digits[1] & 8));
	
}

///BEGIN OTHER CHIP STUFF

// 
// make explicit send to IC inside readandwrite
// program other chip to recieve 4 8-bit bytes and send them right back


uint16_t main_counter = 0;
int16_t mycounter16 = 0;
int16_t adc_1 = 0;
int16_t mycounter16In = 0;
int16_t prevcounterOut = 0;
uint8_t mycounter = 0;
uint8_t stopME = 0;


void pullUpOtherChipCS(void)
{
	static HAL_SPI_StateTypeDef spiState;
  spiState	= HAL_SPI_GetState(&hspi1);
	while(spiState != HAL_SPI_STATE_READY)
	{
		spiState	= HAL_SPI_GetState(&hspi1);
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other_IC_CS pin goes high to end conversion
}


int16_t readAndWriteOtherChip(int16_t Audio_In, uint16_t Buffer_Index)
{
	/*
	for (int i = 0; i < 4; i++)
	{
		IC_tx[i] = mycounter;
		mycounter++;
	}
*/
if (stopME == 0)
{
		adc_1 = (int16_t) (ADC_values[0]);
		WRITE_INT16_AS_BYTES(0,mycounter16);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Other_IC_CS pin goes low to start conversion
		//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
		//mycounter16In = READ_BYTES_AS_INT16(2);
	  
}
	return 0;
	
	//mycounter16++;
	//WRITE_FLOAT16_AS_BYTES((2 * Buffer_Index), Audio_In);
	//WRITE_INT16_AS_BYTES((2 * Buffer_Index), mycounter16);
 //  mycounter16++;
	//if (mycounter16 > 1000)
	//{
	//	mycounter16 = 0;
	//}
	//return READ_BYTES_AS_FLOAT16(2 * Buffer_Index);
	//return READ_BYTES_AS_INT16(2 * Buffer_Index);
	
		//int offset = 4;
	//int numParams = 6;
	/*
	for (int i = 0; i < numParams; i++)
	{
		IC_tx[i * 2 + offset] = (ADC_values[i] >> 8);
		IC_tx[i * 2 + offset + 1] = (ADC_values[i] & 255);
	}
	

	if (main_counter == 0)
	{
		mycounter++;
	}
	for(int i = 0; i < IC_BUFFER_SIZE; i++)
	{
		IC_tx[i] = (mycounter + i);
	}
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	Write7SegWave(IC_rx[0]);

	main_counter++;
  return 0;
	*/
}



void sendBytesToOtherIC(uint8_t whichByte, uint8_t data) 
{
	IC_tx[whichByte] = data; 
}

uint8_t readBytesFromOtherIC(uint8_t startingByte)
{
	return IC_rx[startingByte];
}

void WRITE_INT16_AS_BYTES(uint8_t byteNum, int16_t data) { 
	
      sendBytesToOtherIC(byteNum, (data >> 8)); 
	    sendBytesToOtherIC((byteNum + 1), (data & 255)); 
}

int16_t READ_BYTES_AS_INT16(uint8_t byteNum) { 

   int16_t data = ((readBytesFromOtherIC(byteNum) << 8) + (readBytesFromOtherIC(byteNum + 1) & 255));
   return(data); 
}

void WRITE_FLOAT16_AS_BYTES(uint8_t byteNum, float data) { 
   int i;

   for (i = 0; i < 2; i++) 
      sendBytesToOtherIC(i + byteNum, *((uint8_t*)&data + i + 2) ) ; 
}

float READ_BYTES_AS_FLOAT16(uint8_t byteNum) { 
   int i; 
   float data;

   for (i = 2; i < 2; i++) 
      *((uint8_t*)&data + i + 2) = readBytesFromOtherIC(i + byteNum);

   return(data); 
}


////END OTHER CHIP STUFF

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	//pullUpOtherChipCS();
	audioTick(HALF_BUFFER_SIZE);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	//pullUpOtherChipCS();
  audioTick(0);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	//if it's SPI1, that's the other stm32f4 IC, ready for new data.
	/*
	if(hspi == &hspi1)
	{
		//HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE, IC_TIMEOUT);
		//nothing should be necessary, since the buffer should be circular
		//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
		//for (int i = 0; i < HALF_IC_BUFFER_SIZE; i++)
		//{
		//	IC_tx[i] = ICbufferTx[i];
		//}
	}
	*/
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	//if it's SPI1, that's the other stm32f4 IC, ready for new data.
	if(hspi == &hspi1)
	{
			int16_t dummy1 = 0;
     uint16_t dummy2 = 0;	
		//HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE, IC_TIMEOUT);
		//nothing should be necessary, since the buffer should be circular
		//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
	hspi1.State = HAL_SPI_STATE_READY;	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
		
		mycounter16In = READ_BYTES_AS_INT16(0);
		//Write7SegWave(mycounter16In);
		
		if (mycounter16In != prevcounterOut)
		{
			//stopME = 1;
		}
		prevcounterOut = mycounter16; 
		mycounter16++;
		
      //OtherICBufferIndexNum = 0;
	    //start_Other_IC_Communication();
		readAndWriteOtherChip(dummy1, dummy2);
		//for (int i = 0; i < HALF_IC_BUFFER_SIZE; i++)
		//{
		//	IC_tx[i + HALF_IC_BUFFER_SIZE] = ICbufferTx[i + HALF_IC_BUFFER_SIZE];
		//}
	}
	if(hspi == &hspi2)
	{
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high
		readExternalADC();
	}
}

void start_Other_IC_Communication(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Other_IC_CS pin goes low to start conversion
	//HAL_Delay_us(1);
	for (int i = 0; i < 10; i++)
	{
	}
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	Write7SegWave(HAL_SPI_GetError(hspi));
}

void getXY_Touchpad(void) {
  // retreive values from the XY touchpad
	// I2C address = 9, 1000k, 
	HAL_I2C_Master_Receive(&hi2c1, 18, myTouchpad, i2cDataSize2, IC_TIMEOUT);
}

 void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//Write7SegWave((myTouchpad[0] / 4));
	//HAL_I2C_Master_Receive_DMA(&hi2c1, 18, myTouchpad, i2cDataSize2);
}

