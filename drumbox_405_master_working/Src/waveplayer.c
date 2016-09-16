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

#define AUDIO_BUFFER_SIZE             256 //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)

#define NUM_PARAMS 3
//#define ADC_DECAY_SINE 3
//#define ADC_DECAY_NOISE 4
#define ADC_FEEDBACK 0
#define ADC_FREQ 1
#define ADC_DELAY 2

// Sine 
tCycle sin1; 

float mParamInc[NUM_PARAMS];
float destParamValue[NUM_PARAMS];
float currParamValue[NUM_PARAMS];
int8_t dirParamInc[NUM_PARAMS];

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
float gainBoost = 3.0f;

int N = 0;
float noiseGain = 1.f;
float m_input0 =0.f;
float m_input1 = 0.f;
float	m_output0 = 0.f;

float m_input0_f2 =0.f;
float m_input1_f2 = 0.f;
float	m_output0_f2 = 0.f;
float	lpOut_f2 = 0.f;

float AudioGateVal = 0.05f;
int16_t computed_samples[HALF_BUFFER_SIZE];

#define FEEDBACK_LOOKUP 5
#define DELAY_LOOKUP 4
float FeedbackLookup[FEEDBACK_LOOKUP] = { 0.0f, 0.8f, .999f, 1.0f, 1.001f };
//float DelayLookup[DELAY_LOOKUP] = { 16000.f, 1850.f, 180.f, 40.f };
float DelayLookup[DELAY_LOOKUP] = { 50.f, 180.f, 1400.f, 16300.f };

uint32_t myRandomNumber = 0;

HAL_StatusTypeDef myStatus = HAL_ERROR;
		
/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;

/* Audio Play Start variable. 
   Defined as external in main.c*/
__IO uint32_t AudioPlayStart = 0;


/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
int16_t Send_Buffer[AUDIO_BUFFER_SIZE];
int16_t Receive_Buffer[AUDIO_BUFFER_SIZE];
int32_t myTimeout = 200;

//for wavetable synth
float phasor = 0.f;
#define INIT_FREQ 110.f

float phaseInc = (INIT_FREQ / SAMPLE_RATE);

float scalar[3];
uint8_t ADC_out[2];
uint8_t ADC_in[2];
#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

/* Private function prototypes -----------------------------------------------*/
// PROTOTYPES
float wavetableSynth(void);
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
void changePhaseInc(float);
float myProcess(float);
float pinkNoise(void);
float whiteNoise(void);
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

void changePhaseInc(float freq)
{
	phaseInc = (freq / SAMPLE_RATE);
}

void StartAudio(void)
{ 
	HAL_Delay(100);
	
	// initialize audio 
	tCycleInit(&sin1, SAMPLE_RATE, sinewave, SINE_TABLE_SIZE);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init();
	HAL_Delay(100);
	//set up the scalars that set the decay times for the envelopes
	initScalars();
	// time to set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	myStatus = HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
}



void updatePhase(void) {
	phasor += phaseInc;

	if (phasor >= 1.0f)
		phasor -= 1.0f;
}

	//BEAUTIFUL SINE WAVE GENERATOR IMPORTANT
float wavetableSynth(void)
{

	int intPart;
	float fracPart, samp0, samp1, single_samp, temp;

	//for (i = 0; i < BUFFER_LENGTH; i++)
	//{
		//linear interpolation
		temp = phasor * SINE_TABLE_SIZE;
		intPart = temp;
		fracPart = temp - intPart;
		samp0 = sinewave[intPart];
		if (++intPart >= SINE_TABLE_SIZE)
			intPart = 0;
		samp1 = sinewave[intPart];
		single_samp = (samp0 + (samp1 - samp0) * fracPart); //scale amplitude down

		updatePhase();
		return single_samp;
	//}
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

float myProcess(float AudioIn)
{
	float sample;
  float envGain[2];
	int m = 0;
	scalar[0] =( powf( 0.5f, (1.0f/(((float)ADC_values[4]) * INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	scalar[1] =( powf( 0.5f, (1.0f/(((float)ADC_values[5]) * INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	
	//set frequency of sine and delay
	//sin1.freq(&sin1, MtoF((currParamValue[ADC_FREQ]) * 109.0f + 25.f));
	phaseInc = MtoF((currParamValue[ADC_FREQ]) * 109.0f + 25.f) * INV_TWO_TO_15;
	setDelay(currParamValue[ADC_DELAY]);
	
	AudioGateVal = ((float)ADC_values[3]) * INV_TWO_TO_12 * 0.2f;
	env_detector_thresh = AudioGateVal;
	if (AudioIn < AudioGateVal)
	{
		AudioIn = 0;
	}
	envGain[0] = adc_env_detector(AudioIn, 0);
  envGain[1] = adc_env_detector(AudioIn, 1);
	

	sample = ((KSprocess(AudioIn) * 0.7f) + AudioIn * 0.8f);
	sample += (0.8f * ((wavetableSynth() * envGain[0] * 0.8f) + (whiteNoise() * envGain[1] * 0.18f)));
  sample = highpass(FastTanh2Like4Term(sample * gainBoost));

	
	//sample = (sin1.step(&sin1) * envGain[0] * 0.8f);
	//update Parameters
	for (m = 0; m < NUM_PARAMS; m++)
	{
		
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

	}
		
  return sample;
}

float whiteNoise (void)
{
	HAL_RNG_GenerateRandomNumber(&hrng, &myRandomNumber);
	float white = (((float)(myRandomNumber >> 16))- 32768.f) / 32768.f;
	return white;
}


float pinkb0;
float pinkb1;
float pinkb2;
float pinkNoise (void)
{
	//pink noise ///
	
	  float tmp;
	  float startWhite = whiteNoise();
		pinkb0 = 0.99765f * pinkb0 + startWhite * 0.0990460f;
		pinkb1 = 0.96300f * pinkb1 + startWhite * 0.2965164f;
		pinkb2 = 0.57000f * pinkb2 + startWhite * 1.0526913f;
		tmp = pinkb0 + pinkb1 + pinkb2 + startWhite * 0.1848f;
	  return tmp * 0.05f;
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
	for (i = 0; i < NUM_PARAMS; i++)
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
	
	if (thresholdCheck(2))
	{
		newFeedback = interpolateFeedback(ADC_values[2]);
		if (newFeedback != destParamValue[ADC_FEEDBACK])
		{
			setRamp(ADC_FEEDBACK, currParamValue[ADC_FEEDBACK], newFeedback, 50.f);
		}
	}
	ADC_LastRead[2] = ADC_values[2];
	
	if (thresholdCheck(1))
	{
		newFreq =  (ADC_values[1] * INV_TWO_TO_12) * (ADC_values[0] * INV_TWO_TO_12); 
		if (newFreq != destParamValue[ADC_FREQ])
		{
			setRamp(ADC_FREQ, currParamValue[ADC_FREQ], newFreq, 90.f);
		}
	}
	ADC_LastRead[1] = ADC_values[1];

	if (thresholdCheck(0))
	{
		newDelay = interpolateDelay((4096 - ADC_values[0]));
		if (newDelay != destParamValue[ADC_DELAY])
		{
			setRamp(ADC_DELAY, currParamValue[ADC_DELAY], newDelay, 90.f);
		}
	}
	ADC_LastRead[0] = ADC_values[0];
}

/*
param_index is identification and array index for parameter requiring ramping
(ADC_FEEDBACK 0, ADC_FREQ 1, ADC_DELAY 2)
old_value is current/just changed value of input
new_value is new value of input
ms_ramp is milliseconds desired for ramping for specific parameter
*/
void setRamp(uint8_t param_index, float old_value, float new_value, float ms_ramp) 
{
	mParamInc[param_index] = (new_value - old_value)/ms_ramp * INV_SR_MS;
	if (mParamInc[param_index] < 0.0f)
		dirParamInc[param_index] = -1;
	else if (mParamInc[param_index] == 0.0f)
		dirParamInc[param_index] = 0;
	else 
		dirParamInc[param_index] = 1;
	destParamValue[param_index] = new_value;
}

#define RAW_THRESHOLD 0.5f

//ADC_Raw thresholding!
uint8_t thresholdCheck(uint8_t raw_num)
{
	float diff = (float)(ADC_values[raw_num] - ADC_LastRead[raw_num]);
	if (diff >= RAW_THRESHOLD || diff <= -RAW_THRESHOLD)
	{
		return 1;
	}
	return 0;
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




