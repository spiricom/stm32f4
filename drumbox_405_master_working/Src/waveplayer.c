/**
This is the Snyderphonics DrumBox synthesis code. 
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "i2s.h"
#include "i2c.h"
#include "dma.h"
#include "codec.h"
#include "rng.h"
#include "stm32f4xx_hal_i2s_ex.h"
#include "math.h"
#include "spi.h"

#include "main.h"
#include "waveplayer.h"
#include "wavetables.h"
#include "audiounits.h"
#include "utilities.h" // three of i and one of you, what am i?

#define AUDIO_BUFFER_SIZE             32 //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)
#define NUM_SMOOTHED_PARAMS 3
#define NUM_KNOBS 6

#define DELAY_BUFFER_LENGTH 16384

/* Ping-Pong buffer used for audio play */
int16_t Send_Buffer[AUDIO_BUFFER_SIZE];
int16_t Receive_Buffer[AUDIO_BUFFER_SIZE];

float smoothedParams[NUM_SMOOTHED_PARAMS];
float delayBuffer1[DELAY_BUFFER_LENGTH];



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


int whichKnob = 0;
float feedbacksamp = 0.f;

float newFreq = 0.0f;
float newDelay = 0.0f;
float newFeedback = 0.0f;
float gainBoost = 0.3f;

float m_input1 = 0.f;
float	m_output0 = 0.f;


#define FEEDBACK_LOOKUP_SIZE 5
#define DELAY_LOOKUP_SIZE 4
float FeedbackLookup[FEEDBACK_LOOKUP_SIZE] = { 0.0f, 0.8f, .999f, 1.0f, 1.001f };
//float DelayLookup[DELAY_LOOKUP_SIZE] = { 16000.f, 1850.f, 180.f, 40.f };
float DelayLookup[DELAY_LOOKUP_SIZE] = { 50.f, 180.f, 1400.f, 16300.f };

uint8_t ADC_out[2];
uint8_t ADC_in[2];
#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

/* Private function prototypes -----------------------------------------------*/
// PROTOTYPES
float ksTick(float);
void readExternalADC(void);
float audioProcess(float);
void audioTick(uint16_t buffer_offset);
float interpolateDelayControl(float raw_data);
float interpolateFeedback(float raw_data);

void sendBytesToOtherIC(uint8_t startingByte, uint8_t data);
uint8_t readBytesFromOtherIC(uint8_t startingByte);
void WRITE_FLOAT_AS_BYTES(uint8_t byteNum, float data);
float READ_BYTES_AS_FLOAT(uint8_t byteNum);

/* Private functions ---------------------------------------------------------*/

// Returns random floating point value [0.0,1.0)
float randomNumber(void) {
	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (((float)(rand >> 16))- 32768.f) * INV_TWO_TO_15;
	return num;
}

void audioInit(void)
{ 
	HAL_Delay(100);
	
	// initialize audio 
	tCycleInit(&sin1, SAMPLE_RATE, sinewave, SINE_TABLE_SIZE);
	tNoiseInit(&noise1, SAMPLE_RATE, &randomNumber, NoiseTypePink);
	tRampInit(&rampFeedback, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampSineFreq, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampDelayFreq, SAMPLE_RATE, 10.0f, 1);
	tHighpassInit(&highpass1, SAMPLE_RATE, 20.0f);
	tHighpassInit(&highpass2, SAMPLE_RATE, 20.0f);
	tEnvelopeFollowerInit(&envFollowNoise, 0.05, 0.0f);
	tEnvelopeFollowerInit(&envFollowSine, 0.05, 0.0f);
	tDelayInit(&delay1,delayBuffer1);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init();
	HAL_Delay(100);

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
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
	
	float sample = ((ksTick(audioIn) * 0.7f) + audioIn * 0.8f);
	sample += (0.8f * ((sin1.tick(&sin1) * envFollowSine.tick(&envFollowSine,audioIn) * 0.8f) + (noise1.tick(&noise1) * envFollowNoise.tick(&envFollowNoise,audioIn) * 0.5f)));
	sample *= gainBoost;
  sample = highpass1.tick(&highpass1, shaper1[(uint16_t)((sample+1.0f)*0.5f * TWO_TO_15)]);

	//update Parameters
	smoothedParams[ControlParameterFeedback] = rampFeedback.tick(&rampFeedback);
	smoothedParams[ControlParameterFrequency] = rampSineFreq.tick(&rampSineFreq);
	smoothedParams[ControlParameterDelay] = rampDelayFreq.tick(&rampDelayFreq);

  return sample;
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
		feedbacksamp = m_output0;
		//float samp = tanh[(uint16_t)((((highpass2.tick(&highpass2,feedbacksamp)) + 1.0f) * 0.5f) * TWO_TO_15)];
		return shaper1[(uint16_t)((((highpass2.tick(&highpass2,feedbacksamp)) + 1.0f) * 0.5f) * TWO_TO_15)];
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

float interpolateFeedback(float raw_data)
{
	float scaled_raw = raw_data * INV_TWO_TO_12;
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
	
	if (knobnum == ControlParameterFeedback)
	{
		newFeedback = interpolateFeedback(ADC_values[ControlParameterFeedback]);
		rampFeedback.setDest(&rampFeedback,newFeedback);
	}
	else if (knobnum == ControlParameterFrequency)
	{
		newFreq =  (ADC_values[ControlParameterFrequency] * INV_TWO_TO_12) /* * (ADC_values[0] * INV_TWO_TO_12)*/; 
		rampSineFreq.setDest(&rampSineFreq,newFreq);
	}
	else if (knobnum == ControlParameterDelay)
	{
		newDelay = interpolateDelayControl((4096 - ADC_values[ControlParameterDelay]));
		rampDelayFreq.setDest(&rampDelayFreq,newDelay);
	}
	whichKnob++;
	if (whichKnob >= NUM_KNOBS)
	{
		whichKnob = 0;
	}
}

void sendBytesToOtherIC(uint8_t startingByte, uint8_t data) 
{
}

uint8_t readBytesFromOtherIC(uint8_t startingByte)
{
	return 0;
}

void WRITE_FLOAT_AS_BYTES(uint8_t byteNum, float data) { 
   int i;

   for (i = 0; i < 4; i++) 
      sendBytesToOtherIC(i + byteNum, *((uint8_t*)&data + i) ) ; 
}

float READ_BYTES_AS_FLOAT(uint8_t byteNum) { 
   int i; 
   float data;

   for (i = 0; i < 4; i++) 
      *((uint8_t*)&data + i) = readBytesFromOtherIC(i + byteNum);

   return(data); 
}


void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	audioTick(HALF_BUFFER_SIZE);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
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




