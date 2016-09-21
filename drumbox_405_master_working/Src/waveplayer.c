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

#define AUDIO_BUFFER_SIZE             128 //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)

#define NUM_PARAMS_SMOOTH 3
#define ADC_FEEDBACK 0
#define ADC_FREQ 1
#define ADC_DELAY 2

// Sine 
tCycle sin1; 

// Noise 
tNoise noise1;

// Ramp
tRamp rampFeedback;
tRamp rampSineFreq;
tRamp rampDelayFreq;

float mParamInc[NUM_PARAMS_SMOOTH];
float destParamValue[NUM_PARAMS_SMOOTH];
float currParamValue[NUM_PARAMS_SMOOTH];
int8_t dirParamInc[NUM_PARAMS_SMOOTH];

float delayed_samples[DELAY_BUF_LENGTH];

float feedbacksamp = 0.f;
float oldFeedbackSamp = 0.f;
float historyCrossfade = 0.f;
int crossfade_count;
float	our_tempdelay = 100.f;
float	our_lastdelay = 100.f;
	const float qm1 = 1.f / 0.33f;
float	envOutput[3] = {0.0f};
float	envGain[3] = {0.0f};
float env_detector_thresh = .05f;
float prevEnvGain = 0.f;
int16_t ledVal = 0;
const float gateGain = 0.024f;

float newFreq = 0.0f;
float newDelay = 0.0f;
float newFeedback = 0.0f;
float newDecay = 0.0f;
float newDecaySine, newDecayNoise, newCF_DelaySine, newCF_NoiseSine;
float gainBoost = 0.3f;

int N = 0;
float noiseGain = 1.f;
float m_input0 =0.f;
float m_input1 = 0.f;
float	m_output0 = 0.f;

float m_input0_f2 =0.f;
float m_input1_f2 = 0.f;
float	m_output0_f2 = 0.f;
float	lpOut_f2 = 0.f;

int16_t computed_samples[HALF_BUFFER_SIZE];

#define FEEDBACK_LOOKUP 5
#define DELAY_LOOKUP 4
float FeedbackLookup[FEEDBACK_LOOKUP] = { 0.0f, 0.8f, .999f, 1.0f, 1.001f };
//float DelayLookup[DELAY_LOOKUP] = { 16000.f, 1850.f, 180.f, 40.f };
float DelayLookup[DELAY_LOOKUP] = { 50.f, 180.f, 1400.f, 16300.f };

/* Ping-Pong buffer used for audio play */
int16_t Send_Buffer[AUDIO_BUFFER_SIZE];
int16_t Receive_Buffer[AUDIO_BUFFER_SIZE];

float scalar[3];
uint8_t ADC_out[2];
uint8_t ADC_in[2];
#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

/* Private function prototypes -----------------------------------------------*/
// PROTOTYPES
float envelope(float, float, float);
float process(void);
void  initWaveguide(void);
float clip(float n, float lower, float upper);
float MtoF(float midi);
float KSprocess(float);
float adc_env_detector(float, int);	
float get_adc_average(void);
void  fillBufferAndRamp(int, int);
float unit16scaling(float);
void  setPeakFilter(void);
float highpass(float);
float highpass2(float);
float gate(float);
float FastTanh2Like4Term(float);
float FastTanh2Like1Term(float);
void getADC_Raw(void);
void setRamp(uint8_t param_index, float old_value, float new_value, float ms_ramp);
void readExternalADC(int which);
void sendBytesToOtherIC(uint8_t startingByte, uint8_t data);
uint8_t readBytesFromOtherIC(uint8_t startingByte);
void WRITE_FLOAT_AS_BYTES(uint8_t byteNum, float data);
float READ_BYTES_AS_FLOAT(uint8_t byteNum);
void setDelay(float);
void updatePhase(void);
float myProcess(float);
float pinkNoise(void);
void 	initScalars(void);
uint8_t thresholdCheck(uint8_t raw_num);
float interpolateFeedback(float raw_data);
float interpolateDelay(float raw_data);
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


void StartAudio(void)
{ 
	HAL_Delay(100);
	
	// initialize audio 
	tCycleInit(&sin1, SAMPLE_RATE, sinewave, SINE_TABLE_SIZE);
	tNoiseInit(&noise1, SAMPLE_RATE, &randomNumber, NoiseTypePink);
	tRampInit(&rampFeedback, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampSineFreq, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampDelayFreq, SAMPLE_RATE, 10.0f, 1);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init();
	HAL_Delay(100);
	//set up the scalars that set the decay times for the envelopes
	initScalars();
	// time to set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
}

float testTemp;
int whichknob = 0;
void fillBufferWithInputProcess(uint8_t buffer_offset)
{
      getADC_Raw();
			uint16_t i = 0;
			int16_t current_sample = 0;    
	    if(buffer_offset == 0)
      {
					for (i = 0; i < (HALF_BUFFER_SIZE); i++)
					{
						if ((i & 1) == 0)
						{
							testTemp = myProcess(Receive_Buffer[i] * INV_TWO_TO_15) * TWO_TO_15;
							current_sample = (int16_t) testTemp;
						}
						Send_Buffer[i] = current_sample;
					}
      }
      
      if(buffer_offset == 1)
      {
					for (i = 0; i < (HALF_BUFFER_SIZE); i++)
					{
						if ((i & 1) == 0)
						{
 							testTemp = myProcess((float) (Receive_Buffer[HALF_BUFFER_SIZE + i] * INV_TWO_TO_15)) * TWO_TO_15;
							current_sample = (int16_t) testTemp;
						}
						Send_Buffer[HALF_BUFFER_SIZE + i] = current_sample;
					}
      } 
			readExternalADC(whichknob);
			whichknob++;
			if (whichknob > 6)
			{
				whichknob = 0;
			}
}

int m = 0;
float myProcess(float AudioIn)
{
	float sample;
  float envGain[2];
	
	scalar[0] =( powf( 0.5f, (1.0f/(((float)ADC_values[4]) * INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	scalar[1] =( powf( 0.5f, (1.0f/(((float)ADC_values[5]) * INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	
	//set frequency of sine and delay
	sin1.freq(&sin1, MtoF((currParamValue[ADC_FREQ]) * 109.0f + 25.f));
	setDelay(currParamValue[ADC_DELAY]);
	
	env_detector_thresh = ((float)ADC_values[3]) * INV_TWO_TO_12 * 0.2f;
	float clippedSample;
	if (AudioIn < env_detector_thresh) 
		clippedSample = 0.f;
	else 
		clippedSample = AudioIn;
	
	envGain[0] = adc_env_detector(clippedSample, 0);
	envGain[1] = adc_env_detector(clippedSample, 1);
	
	
	sample = ((KSprocess(AudioIn) * 0.7f) + AudioIn * 0.8f);
	sample += (0.8f * ((sin1.step(&sin1) * envGain[0] * 0.8f) + (noise1.step(&noise1) * envGain[1] * 0.5f)));
  sample = highpass(FastTanh2Like4Term(sample * gainBoost));

	//update Parameters
	currParamValue[0] = rampFeedback.step(&rampFeedback);
	currParamValue[1] = rampSineFreq.step(&rampSineFreq);
	currParamValue[2] = rampDelayFreq.step(&rampDelayFreq);
	/*
	if ((currParamValue[m] >= destParamValue[m]) && (dirParamInc[m] == 1)) 
	{
		mParamInc[m] = 0.0f;
		currParamValue[m] = destParamValue[m];
	}
	else if ((currParamValue[m] <= destParamValue[m]) && (dirParamInc[m] == -1))
	{
		mParamInc[m] = 0.0f;
		currParamValue[m] = destParamValue[m];
	}
	else if (dirParamInc[m] == 0)
	{
		mParamInc[m] = 0.0f;
		currParamValue[m] = destParamValue[m];
	}
	else 
	{
		currParamValue[m] += mParamInc[m];
	}
	m++;
	if (m >= NUM_PARAMS_SMOOTH) m = 0;
	*/
  return sample;
}


float MtoF(float midi)
{
	return powf(2.f, (midi - 69.f) * 0.0833f) * 440.f;
}

void initScalars(void)
{
  scalar[0] = ( powf( 0.5f, (1.0f/(0.6f * (float)SAMPLE_RATE))));
	scalar[1] = ( powf( 0.5f, (1.0f/(0.1f * (float)SAMPLE_RATE))));
	scalar[2] = ( powf( 0.5f, (1.0f/(0.2f * (float)SAMPLE_RATE))));
	uint8_t i = 0;
	for (i = 0; i < NUM_PARAMS_SMOOTH; i++)
	{
		dirParamInc[i] = 0.0f;
	}
}

float sabs(float a)
{
	int b=(*((int *)(&a)))&0x7FFFFFFF;
	return *((float *)(&b));
}

float tanh2(float x)
{
	float a = sabs(x);
	a = 6 + a *(6 + a * (3 + a));
	return ((x<0) ? -1 : 1) * (a-6)/(a+6); 
}


void getADC_Raw(void)
{
		newFeedback = interpolateFeedback(ADC_values[2]);
		rampFeedback.setDest(&rampFeedback,newFeedback);

		newFreq =  (ADC_values[1] * INV_TWO_TO_12) /* * (ADC_values[0] * INV_TWO_TO_12)*/; 
		rampSineFreq.setDest(&rampSineFreq,newFreq);

		newDelay = interpolateDelay((4096 - ADC_values[0]));
		rampDelayFreq.setDest(&rampDelayFreq,newDelay);
}


#define ADC_DECAY_SINE 0
#define ADC_DECAY_NOISE 1

float a0;
float a1;
float b1;


float xs = 0.0f;
float ys = 0.0f;
float R = (1.f-(126.f/SAMPLE_RATE));
float y = 0.0f;

float highpass(float x)
{
y = x - xs + R * ys;
	ys = y;
	xs = x;
	return y;
}

float xs2 = 0.0f;
float ys2 = 0.0f;
float R2 = (1.f-(126.f/SAMPLE_RATE));
float y2 = 0.0f;

float highpass2(float x2)
{
	
  y2 = x2 - xs2 + R2 * ys2;
	ys2 = y2;
	xs2 = x2;
	return y2;
}

float gate(float input)
{
	if (input > gateGain)
	{
		return input;
	}
	else return 0.0f;
}

float adc_env_detector(float input, int whichone) {
	// halfLife = time in seconds for output to decay to half value after an impulse


	if(input < 0.0f )
		input = -input;  /* Absolute value. */
	
	
		if ((input >= envOutput[whichone]) && (input > env_detector_thresh))
		{
			 /* When we hit a peak, ride the peak to the top. */
			 return envOutput[whichone] = input;
		}
		else
		{
			 /* Exponential decay of output when signal is low. */
			 envOutput[whichone] = envOutput[whichone] * scalar[whichone];
			 /*
			 ** When current gets close to 0.0, set current to 0.0 to prevent FP underflow
			 ** which can cause a severe performance degradation due to a flood
			 ** of interrupts.
			 */
			 if( envOutput[whichone] < VERY_SMALL_FLOAT ) 
				 return envOutput[whichone] = 0.0f;
		}
		return envOutput[whichone];

}


float FastTanh2Like4Term(float Input)
{
	float a, b;
	if (Input < 0.0f)
	{
		a = Input * -1.0f;
	}
	else
	{
		a = Input;
	}
	b = 12.0f + a * (6.0f + a * (3.0f + a));
	return (Input * b) / (a * b + 24.0f);

}

float FastTanh2Like1Term(float Input)
{
	float a;
	if (Input < 0.0f)
	{
		a = Input * -1.0f;
	}
	else
	{
		a = Input;
	}
	return (Input / (a + 3.0f));
}

float interpolateDelay(float raw_data)
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

void readExternalADC(int which)
{

	uint8_t knobnum;
	uint16_t knobval12bit;
	uint16_t knobsendbyte;
	uint8_t ADC_out[2];

	//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
	knobsendbyte = 6208 + (which << 7);
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
}


float nextOutput;
int doNextOut = 1;
int inPoint = 0;
float length = DELAY_BUF_LENGTH;
uint32_t outPoint;
float delay;
float alpha, omAlpha;
uint32_t outPoint;

void setDelay(float theDelay)
{
  float outPointer;

	outPointer = inPoint - theDelay;  // read chases write
	delay = theDelay;

  while (outPointer < 0)
    outPointer += length; // modulo maximum length

  outPoint = (uint32_t) outPointer;  // integer part
  alpha = outPointer - outPoint; // fractional part
  omAlpha = (float) 1.0 - alpha;
}


float nextOut(void)
{
  if ( doNextOut ) {
    // First 1/2 of interpolation
    nextOutput = delayed_samples[outPoint] * omAlpha;
    // Second 1/2 of interpolation
    if (outPoint+1 < length)
      nextOutput += delayed_samples[outPoint+1] * alpha;
    else
      nextOutput += delayed_samples[0] * alpha;
    doNextOut = 0;
  }
  return nextOutput;
}

float tick(float sample)
{
	float output_sample;
  delayed_samples[inPoint++] = sample;

  // Increment input pointer modulo length.
  if (inPoint == length)
    inPoint -= length;

  output_sample = nextOut();
  doNextOut = 1;

   // Increment output pointer modulo length.
  if (++outPoint >= length)
    outPoint -= length;

  return output_sample;
}

float KSprocess(float noise_in)
{
		float temp_sample;
	  
		temp_sample = noise_in + (feedbacksamp * currParamValue[ADC_FEEDBACK]);
		
		feedbacksamp = tick(temp_sample);
				
	  //simple one-zero lowpass filter (moving average)
		m_output0 = 0.5f * m_input1 + 0.5f * feedbacksamp;
		m_input1 = feedbacksamp;
		feedbacksamp = m_output0;
		
		return FastTanh2Like4Term(highpass2(feedbacksamp));
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
		fillBufferWithInputProcess(1);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    fillBufferWithInputProcess(0);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}




