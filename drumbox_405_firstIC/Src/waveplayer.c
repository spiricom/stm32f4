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
#include "wavetables.h"
#include "utilities.h"
#include "audiounits.h"
#include "waveplayer.h"




#define TEST_LED_ENABLED 0
#define USE_NEW_FEEDBACK 0

#define AUDIO_BUFFER_SIZE             64 //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)
#define NUM_SMOOTHED_PARAMS 3
#define NUM_KNOBS 6
#define NUM_ADC_VALUES 10
#define CAT_BUFFERSIZE 2
#define TIMEOUT 10
#define NUM_LEDS 8

#define USE_TOP_RIGHT_FOR_NOISE_DECAY 1

#define IC_BUFFER_SIZE 4
#define HALF_IC_BUFFER_SIZE (IC_BUFFER_SIZE / 2)
#define IC_TIMEOUT 200

#define DELAY_BUFFER_LENGTH 16384

#define NUM_FB_DELAY_TABLES 8

/* Ping-Pong buffer used for audio play */
int16_t audioOutBuffer[AUDIO_BUFFER_SIZE];
int16_t audioInBuffer[AUDIO_BUFFER_SIZE];

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
uint8_t I2CReset_buffer[1] = {0x06};
uint8_t I2C_init_command[3] = {0x58, 0x90, 0x00};
uint16_t ADC_values[NUM_ADC_VALUES];

// 0 = ADC, 1 = LED
uint8_t whichSPI2 = 0;

#define NUM_BYTES_TO_SEND 8
uint8_t externalDACOutputBuffer[NUM_BYTES_TO_SEND];
uint8_t externalDACOutputBufferTX[NUM_BYTES_TO_SEND];

// Sine 
tCycle sin1; 

// Noise 
tNoise noise1;

// Ramp
tRamp rampFeedback;
tRamp rampSineFreq;
tRamp rampDelayFreq;
tRamp rampKSGain; 
tRamp rampInputGain;

// Highpass
tHighpass highpass1;
tHighpass highpass2; 

// Envelope follower
tEnvelopeFollower envFollowNoise;
tEnvelopeFollower envFollowTrigOut;
tEnvelopeFollower envFollowSine;

// Delay 
tDelay delay1;

// SVF
tSVF svf1; 

// Gain
float masterGain = 0.8f;
float sineGain = 0.7f;
float noiseGain = 2.0f; 
float noiseFilterGain = 0.5f;
float ksGain = 0.8f;

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

uint8_t XYLED[2]; 

uint8_t ADC_Tx_Ready = 0;
uint8_t XY_Tx_Ready = 0;



uint8_t CAT_LED[CAT_BUFFERSIZE];
uint8_t JUNK_LED[CAT_BUFFERSIZE];

#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

/* Private function prototypes -----------------------------------------------*/
// PROTOTYPES

// External Dac communication
void DMA1_externalDACInit(void);
void DMA1_externalDACSend(void);

// IC communication functions
void communicateWithOtherIC(void);
void start_Other_IC_Communication(void);
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
void setTrigOut(uint8_t set);
float clip(float min, float val, float max);

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
	//for (int i = 0; i < 10; i++);
	HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)CAT_LED, (uint8_t *)JUNK_LED,  CAT_BUFFERSIZE);
	
}

void setTrigOut(uint8_t set) {
	if (set) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}
	
#define I2C1_TEST_ENABLED 0
void audioInit(void)
{ 

	int16_t dummy1 = 0;
	uint16_t dummy2 = 0;
	DMA1_externalDACInit();
	// Initialize feedback lookup table data.
	float fbdp = 0.0002604165f; //min feedback delay period
	for (int i = 0; i < NUM_FB_DELAY_TABLES; i++) {
		feedbackDelayPeriod[i] = fbdp;
		fbdp *= 2.0f;
	}

	// Initialize cycle.
	// initialize audio 
	tCycleInit(&sin1, SAMPLE_RATE, sinewave, SINE_TABLE_SIZE);
	tNoiseInit(&noise1, SAMPLE_RATE, &randomNumber, NoiseTypeWhite);
	tRampInit(&rampFeedback, SAMPLE_RATE, 10.0f, 1);
	tRampInit(&rampSineFreq, SAMPLE_RATE, 12.0f, 1);
	tRampInit(&rampDelayFreq, SAMPLE_RATE, 20.0f, 1);
	tRampInit(&rampKSGain, SAMPLE_RATE, 5.0f, 1);
	tRampInit(&rampInputGain, SAMPLE_RATE, 5.0f, 1);
	tHighpassInit(&highpass1, SAMPLE_RATE, 20.0f);
	tHighpassInit(&highpass2, SAMPLE_RATE, 45.0f);
	tEnvelopeFollowerInit(&envFollowNoise, 0.001f, 0.0f);
	tEnvelopeFollowerInit(&envFollowSine, 0.001f, 0.0f);
	tEnvelopeFollowerInit(&envFollowTrigOut, 0.01f, 0.99f);
	tDelayInit(&delay1,delayBuffer1);
	tSVFInit(&svf1,SAMPLE_RATE,SVFTypeLowpass, 2000, 1.0f);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high to make sure it's not selected
	HAL_Delay(100);
	
#if I2C1_TEST_ENABLED
	while(1) {
		DMA1_externalDACSend();
		HAL_Delay(100);
	}
#endif

#if TEST_LED_ENABLED
	int x,y;
	int timer = 0;
	int toggle = 0;
	while(1) {
		
		setXYLED(x,y);
		
		
		if (timer++ > 100000) {
			toggle = !toggle;
			setTrigOut(toggle);
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
	//start_Other_IC_Communication();
	
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)&audioOutBuffer[0], (uint16_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
  
	// SPI1 Other Chip communication Initial Call
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Other IC CS pin goes high to allow conversion start
	//HAL_Delay(1); 
	//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE); // removed (uint8_t *) casts before IC_tx and IC_rx
	
	whichSPI2 = 0;
	// SPI2 LED Initial Call
	
	setXYLED(3,3);
	
	
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

uint16_t dacCount1 = 0;
uint16_t dacCount2 = 0;
uint16_t dacCount3 = 0;
uint16_t dacCount4 = 0;

float FM_in; 
int count = 0;
void audioTick(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int16_t current_sample = 0;  

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
		getXY_Touchpad();
	}
	
	newDelay = interpolateDelayControl((4096 - (myTouchpad[1] * 16)));
	rampDelayFreq.setDest(&rampDelayFreq,newDelay);
	
	newFreq = ((mtof1[myTouchpad[0] * 16]) * 0.2f);
	rampSineFreq.setDest(&rampSineFreq,newFreq);

	XYLED[0] = 7 - (uint8_t)(myTouchpad[0] * INV_TWO_TO_5);
	XYLED[1] = 7 - (uint8_t)(myTouchpad[1] * INV_TWO_TO_5);
	
	if (XY_Tx_Ready)
	{
		XY_Tx_Ready = 0;
		setXYLED(XYLED[0],XYLED[1]);
	}
	

	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0) {
			current_sample = (int16_t)(audioProcess((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		} else {
			FM_in = (float)(audioInBuffer[buffer_offset + i] * INV_TWO_TO_15);
		}
		audioOutBuffer[buffer_offset + i] = current_sample;
	}
	
	if (ADC_Tx_Ready) 
	{
		ADC_Tx_Ready = 0;
		readExternalADC();
	}
	
}

float f_abs(float in) {
	float out;
	if (in < 0.0f) {
		out = in * -1.0f;
	} else {
		out = in;
	}
	return out;
}


#define DAC_OUT 1
#define TRIG_OUT 1
uint16_t newcount = 0;
float sinefreq, delayfreq;
typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

#define NUM_SAMPS_BETWEEN_TRIG 2048
uint32_t sampSinceTrig = 0;
uint16_t prevValue = 0;

float noiseToEnv, sineToEnv, envSine1;
uint16_t envIntNoise, envIntSine;
uint16_t envNoise1;
float sineEnv, noiseEnv;
float sample;
float audioProcess(float audioIn) {
	
	//gate the audio input with a ramped gain
	// alternative to custom f_abs is fabs in the math.h library?
	float absAudioIn = f_abs(audioIn);
	if (absAudioIn < 0.0001f) {
		rampInputGain.setDest(&rampInputGain, 0.0f);
	} 
	if (absAudioIn > 0.00015f) {
		rampInputGain.setDest(&rampInputGain, 1.0f);
	}
	float inputGain = rampInputGain.tick(&rampInputGain);
	audioIn = inputGain * audioIn;
	
	// UPDATE NOISE FILTER Q and FREQ
	float Q = 10.0f * (ADC_values[ControlParameterNoiseWidth] * INV_TWO_TO_12) + 0.05f;
	noiseFilterGain = 0.1f*(1.0f - (Q/10.0f))+0.025f;
	svf1.setQ(&svf1, Q);
	svf1.setFreq(&svf1, ADC_values[ControlParameterNoiseCutoff]); 
	
	// SET NOISE DECAY COEFF
#if USE_TOP_RIGHT_FOR_NOISE_DECAY
	envFollowNoise.decayCoeff(&envFollowNoise,adc1[ADC_values[ControlParameterNoiseDecay]]);
#else
	envFollowNoise.decayCoeff(&envFollowNoise,adc1[ADC_values[ControlParameterDive]]);
#endif
	
	// SET SINE DECAY COEFF
  envFollowSine.decayCoeff(&envFollowSine,adc1[ADC_values[ControlParameterSineDecay]]);

	// UPDATE FEEDBACK 
	newFeedback = interpolateFeedback(ADC_values[ControlParameterFeedback]);
	rampFeedback.setDest(&rampFeedback,newFeedback);
	smoothedParams[SmoothedParameterFeedback] = rampFeedback.tick(&rampFeedback);
	
	// UPDATE SINE FREQ
	sin1.freq(&sin1, rampSineFreq.tick(&rampSineFreq) + (FM_in * 1000.0f));
	
	// UPDATE DELAY PERIOD
	smoothedParams[SmoothedParameterDelay] = rampDelayFreq.tick(&rampDelayFreq);
	//delayfreq = smoothedParams[SmoothedParameterDelay] + (1.0f/(((FM_in + 1.0f) * 0.5f) * 1000.0f)); //trying FM delay period -- need to work on this more, it's a number of samples in the array, this is period in seconds, I think
	delay1.setDelay(&delay1, smoothedParams[SmoothedParameterDelay]);
	
	// MIX

	sample = 0.8f * audioIn;
	if (ADC_values[ControlParameterFeedback] > 3) {
		sample += (ksGain * ksTick(audioIn));
	}
	if (ADC_values[ControlParameterSineDecay] > 3) {

		sineEnv = envFollowSine.tick(&envFollowSine,audioIn); 
		sample += (sineGain * sin1.tick(&sin1) * sineEnv);
		
	}
	sample *= .7f;
	
#if USE_TOP_RIGHT_FOR_NOISE_DECAY
	if (ADC_values[ControlParameterNoiseDecay] > 5) {

		noiseEnv = envFollowNoise.tick(&envFollowNoise,audioIn); 
		sample += noiseGain * (noiseFilterGain * svf1.tick(&svf1, noise1.tick(&noise1)) * envFollowNoise.tick(&envFollowNoise,audioIn));
	}
#else 
	if (ADC_values[ControlParameterDive] > 5) {

		noiseEnv = envFollowNoise.tick(&envFollowNoise,audioIn); 
		sample += noiseGain * (noiseFilterGain * svf1.tick(&svf1, noise1.tick(&noise1)) * envFollowNoise.tick(&envFollowNoise,audioIn));
	}
#endif
	
	sample *= masterGain;
	sample *= .5;
	//clip(-1.0f, sample, 1.0f);
	sample = FastTanh2Like1Term(sample);
	//clip(-1.0f, sample, 1.0f);
	//sample = shaper1[(uint16_t)((sample+1.0f)*0.5f * TWO_TO_16_MINUS_ONE)];
  sample = highpass1.tick(&highpass1, sample);

	if (++newcount == 16) {
		newcount = 0;
		

	
		uint16_t sineEnvToDAC = (uint16_t)(sineEnv * TWO_TO_12);
		externalDACOutputBuffer[0] = (sineEnvToDAC >> 8); 
		externalDACOutputBuffer[1] = (sineEnvToDAC & 255);
	
		uint16_t noiseEnvToDAC = (uint16_t)(noiseEnv * TWO_TO_12);
		externalDACOutputBuffer[2] = (noiseEnvToDAC >> 8); 
		externalDACOutputBuffer[3] = (noiseEnvToDAC & 255); 
	
		externalDACOutputBuffer[4] = ((myTouchpad[0] * 16) >> 8);
		externalDACOutputBuffer[5] = (myTouchpad[0] * 16) & 255;

		externalDACOutputBuffer[6] = ((myTouchpad[1] * 16) >> 8);
		externalDACOutputBuffer[7] = (myTouchpad[1] * 16) & 255;
	
		DMA1_externalDACSend();
	}


	
	// TRIGGER
	
	//setTrigOut(peakEnv > 0.05f);
#if TRIG_OUT
	float peakEnv = envFollowTrigOut.tick(&envFollowTrigOut,audioIn);
	if (sampSinceTrig > NUM_SAMPS_BETWEEN_TRIG) {
		
		if (peakEnv > 0.02f) {
			  setTrigOut(1);
				sampSinceTrig = 0;
		} 	
	} else {
		
		sampSinceTrig++;
	}
	
	if (peakEnv < 0.005f) { 
			setTrigOut(0);
	}
#endif
	

	
	return sample;
}

void DMA1_externalDACSend(void) {
	// Start communication with external DAC
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, externalDACOutputBuffer, NUM_BYTES_TO_SEND);

}

void DMA1_externalDACInit(void) {
	// Start communication with external DAC
	//HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, (uint8_t *)&externalDACOutputBufferTX[0], NUM_BYTES_TO_SEND);
	
	//Send a general call to the DAC chip (using i2c address 0 for "general call") that tells it to reset
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 0, I2CReset_buffer, 1);
	HAL_Delay(10);
	//Send initial commands to the CV DAC channels to set their gain and vref stuff correctly
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, I2C_init_command, 3);
	I2C_init_command[0] = 0x5A;
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, I2C_init_command, 3);
	I2C_init_command[0] = 0x5C;
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, I2C_init_command, 3);
	I2C_init_command[0] = 0x5E;
	HAL_I2C_Master_Transmit_DMA(&hi2c1, 192, I2C_init_command, 3);
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
		float temp_sample, out, gain;
	  
		temp_sample = noise_in + (feedbacksamp * smoothedParams[SmoothedParameterFeedback]);
		
		//feedbacksamp = delayTick(temp_sample);
		feedbacksamp = delay1.tick(&delay1, temp_sample);
				
	  //simple one-zero lowpass filter (moving average)
		m_output0 = 0.5f * m_input1 + 0.5f * feedbacksamp;
		m_input1 = feedbacksamp;
		feedbacksamp = clip(-1.0f,highpass2.tick(&highpass2,m_output0),1.0f);
		float absFeedbackSamp;
		if (feedbacksamp < 0.0f)
		{
			absFeedbackSamp = feedbacksamp * -1.0f;
		}
		else
		{
			absFeedbackSamp = feedbacksamp;
		}
		if (absFeedbackSamp < 0.001f) {
			rampKSGain.setDest(&rampKSGain, 0.0f);
		} 
		if (absFeedbackSamp > 0.0015f) {
			rampKSGain.setDest(&rampKSGain, 1.0f);
		}
		gain = rampKSGain.tick(&rampKSGain);
		out = gain * shaper1[(uint16_t)(((feedbacksamp + 1.0f) * 0.5f) * (TWO_TO_16_MINUS_ONE))];
		return out;
}


void readExternalADC(void)
{
	uint8_t knobnum;
	uint16_t knobval12bit;
	uint16_t knobsendbyte;
	uint8_t ADC_out[2];
	
	knobnum = ADC_in[0] >> 4;
	knobval12bit = (ADC_in[1] | ((ADC_in[0] & 15) << 8));
	ADC_values[knobnum] = knobval12bit;
	
	// write to ADC chip (asking for data on next channel)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ADC CS pin goes low to initiate conversion

	//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
	knobsendbyte = 6208 + (whichKnob << 7);
	ADC_out[0] = knobsendbyte >> 8;
	ADC_out[1] = knobsendbyte & 255;
	
	hspi1.State = HAL_SPI_STATE_READY;
	
	//for(int j = 0; j < 10; j++);
	
	HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)ADC_out, (uint8_t *)ADC_in, ADC_BUFFERSIZE);
		
	//for(int j = 0; j < 5000; j++);

/*
	if (knobnum == ControlParameterFeedback){
		
		newFeedback = interpolateFeedback(ADC_values[ControlParameterFeedback]);
		rampFeedback.setDest(&rampFeedback,newFeedback);
	} 
*/
	if (++whichKnob >= NUM_KNOBS)
	{
		whichKnob = 0;
	}
}

float interpolateDelayControl(float raw_data)
{
	float scaled_raw = raw_data * INV_TWO_TO_12;
	if (scaled_raw < 0.4f)
	{
		return (DelayLookup[0] + ((DelayLookup[1] - DelayLookup[0]) * (scaled_raw * 2.5f)));
	}
	else if (scaled_raw < 0.8f)
	{
		return (DelayLookup[1] + ((DelayLookup[2] - DelayLookup[1]) * ((scaled_raw - 0.4f) * 2.5f)));
	}
	else
	{
		return (DelayLookup[2] + ((DelayLookup[3] - DelayLookup[2]) * ((scaled_raw - 0.8f) * 5.0f)));
	}	
}
float out;
uint16_t index1;
float delPer;
float w1,w2,range;
float interpolateFeedback(uint16_t idx) {
	
	index1 = idx;
	float delayPeriod = (smoothedParams[SmoothedParameterDelay] * INV_SAMPLE_RATE);
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


int16_t mycounter16Out = 0;
int16_t mycounter16In = 0;
int16_t prevcounterOut = 0;
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
	if (stopME == 0) {
			WRITE_INT16_AS_BYTES(0,mycounter16Out);
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



void writeOtherICBuffer(uint8_t whichByte, uint8_t data) 
{
	IC_tx[whichByte] = data; 
}

uint8_t readOtherICBuffer(uint8_t startingByte)
{
	return IC_rx[startingByte];
}

void WRITE_INT16_AS_BYTES(uint8_t byteNum, int16_t data) { 
	
      writeOtherICBuffer(byteNum, (data >> 8)); 
	    writeOtherICBuffer((byteNum + 1), (data & 255)); 
}

int16_t READ_BYTES_AS_INT16(uint8_t byteNum) { 

   int16_t data = ((readOtherICBuffer(byteNum) << 8) + (readOtherICBuffer(byteNum + 1) & 255));
   return(data); 
}

void WRITE_FLOAT16_AS_BYTES(uint8_t byteNum, float data) { 
   int i;

   for (i = 0; i < 2; i++) 
      writeOtherICBuffer(i + byteNum, *((uint8_t*)&data + i + 2) ) ; 
}

float READ_BYTES_AS_FLOAT16(uint8_t byteNum) { 
   int i; 
   float data;

   for (i = 2; i < 2; i++) 
      *((uint8_t*)&data + i + 2) = readOtherICBuffer(i + byteNum);

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
#if 0
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi2) {
		
		if (!whichSPI2) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // CAT4016 Latch pin goes high to latch data
			ADC_Tx_Ready = 1;
			whichSPI2 = 1;
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high
			XY_Tx_Ready = 1;
			whichSPI2 = 0;
		}
		
		//readExternalADC();
	}
}
#endif


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	if(hspi == &hspi1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
	} else if (hspi == &hspi2) { 
		if (!whichSPI2) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // CAT4016 Latch pin goes back low to reset it
			for (int i =0; i < 5; i++);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CAT4016 Latch pin goes high to latch data
			ADC_Tx_Ready = 1;
			whichSPI2 = 1;
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high
			XY_Tx_Ready = 1;
			whichSPI2 = 0;
		}
	} else {
			
	}
#if 0
	//if it's SPI1, that's the other stm32f4 IC, ready for new data.
	if(hspi == &hspi1)
	{
		//pullUpOtherChipCS();
		hspi1.State = HAL_SPI_STATE_READY;
			
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
		
		mycounter16In = READ_BYTES_AS_INT16(0); ///this is signed 16 int
		//Write7SegWave(mycounter16In);
		
		if (mycounter16In != prevcounterOut)
		{
			//stopME = 1;
		}
		prevcounterOut = mycounter16Out; 
		mycounter16Out++;
		
    //OtherICBufferIndexNum = 0;
	  //start_Other_IC_Communication();
		
		WRITE_INT16_AS_BYTES(0,mycounter16Out);
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Other_IC_CS pin goes low to start conversion
		
		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
		
	
		//for (int i = 0; i < HALF_IC_BUFFER_SIZE; i++)
		//{
		//	IC_tx[i + HALF_IC_BUFFER_SIZE] = ICbufferTx[i + HALF_IC_BUFFER_SIZE];
		//}
	}

#endif
}

void communicateWithOtherIC(void) {
	//pullUpOtherChipCS();
		hspi1.State = HAL_SPI_STATE_READY;
			
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
		
		mycounter16In = READ_BYTES_AS_INT16(0); ///this is signed 16 int
		//Write7SegWave(mycounter16In);
		
		if (mycounter16In != prevcounterOut)
		{
			//stopME = 1;
		}
		prevcounterOut = mycounter16Out; 
		mycounter16Out++;
		
    //OtherICBufferIndexNum = 0;
	  //start_Other_IC_Communication();
		
		WRITE_INT16_AS_BYTES(0,mycounter16Out);
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Other_IC_CS pin goes low to start conversion
		
		for (int i = 0; i < 10; i++) {
			
		}
		
		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
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
	HAL_I2C_Master_Receive_DMA(&hi2c1, 18, myTouchpad, i2cDataSize2);
}

 void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//Write7SegWave((myTouchpad[0] / 4));
	//HAL_I2C_Master_Receive_DMA(&hi2c1, 18, myTouchpad, i2cDataSize2);
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

