/**
This is the Snyderphonics DrumBox synthesis code. 
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "waveplayer.h"
#include "sinewave.h"
#include "i2s.h"
#include "i2c.h"
#include "dma.h"
#include "codec.h"
#include "rng.h"
#include "stm32f4xx_hal_i2s_ex.h"
#include "math.h"
#include "main.h"
#include "spi.h"
#include "dma.h"

#define AUDIO_BUFFER_SIZE    4  //four is the lowest number that makes sense -- 2 samples for each computed sample (L/R), and then half buffer fills
#define HALF_BUFFER_SIZE      (AUDIO_BUFFER_SIZE/2)
#define IC_BUFFER_SIZE 4
#define HALF_IC_BUFFER_SIZE (IC_BUFFER_SIZE / 2)
#define IC_TIMEOUT 200
#define NUM_PARAMS 3
//#define ADC_DECAY_SINE 3
//#define ADC_DECAY_NOISE 4
#define ADC_FEEDBACK 0
#define ADC_FREQ 1
#define ADC_DELAY 2


/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_i2s3_ext_rx;
extern I2S_HandleTypeDef hi2s3;
extern RNG_HandleTypeDef hrng;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

uint16_t i2cDataSize2 = 2;
uint8_t myTouchpad[2];
uint32_t I2Ctimeout2 = 2000;

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

float m_drive = 0.5f;
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

uint8_t ic_counter = 0;

#define FEEDBACK_LOOKUP 5
#define DELAY_LOOKUP 4
//float FeedbackLookup[FEEDBACK_LOOKUP] = { 0.0f, 0.8f, .999f, 1.0f, 1.001f };
float FeedbackLookup[FEEDBACK_LOOKUP] = { 0.0f, 0.9f, .99f, .999f, 1.0001f };
float FeedbackLookup2[FEEDBACK_LOOKUP] = { 0.0f, 0.99f, .999f, .9999f, 1.0001f };
float FeedbackLookup3[FEEDBACK_LOOKUP] = { 0.0f, 0.999f, .9999f, .99999f, 1.00f };
float FeedbackLookup4[FEEDBACK_LOOKUP] = { 0.0f, 0.9999f, .99999f, .999999f, 1.00f };
//float DelayLookup[DELAY_LOOKUP] = { 16000.f, 1850.f, 180.f, 40.f };
float DelayLookup[DELAY_LOOKUP] = { 50.f, 180.f, 1400.f, 16361.f };

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
uint8_t IC_tx[IC_BUFFER_SIZE];
uint8_t IC_rx[IC_BUFFER_SIZE];
uint8_t ICbufferTx[IC_BUFFER_SIZE*10];
uint8_t ICbufferRx[IC_BUFFER_SIZE*10];
#define ADC_BUFFERSIZE 2
#define TIMEOUT 10

//FOR parameter updates
int m = 0;

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
float highpass3(float);
float gate(float);
float FastTanh2Like4Term(float);
float FastTanh2Like1Term(float);
void getADC_Raw(uint8_t);
void setRamp(uint8_t param_index, float old_value, float new_value, float ms_ramp);
void readExternalADC(int which);
void sendBytesToOtherIC(uint8_t startingByte, uint8_t data);
uint8_t readBytesFromOtherIC(uint8_t startingByte);
void WRITE_FLOAT16_AS_BYTES(uint8_t byteNum, float data);
float READ_BYTES_AS_FLOAT16(uint8_t byteNum);
void WRITE_INT16_AS_BYTES(uint8_t byteNum, int16_t data);
int16_t READ_BYTES_AS_INT16(uint8_t byteNum);
void setDelay(float);
void updatePhase(void);
void changePhaseInc(float);
float wavetableSynth(void);
float myProcess(float);
float pinkNoise(void);
float whiteNoise(void);
void 	initScalars(void);
uint8_t thresholdCheck(uint8_t raw_num);
float interpolateFeedback(float raw_data);
float interpolateDelay(float raw_data);
int16_t readAndWriteOtherChip(int16_t, uint16_t);
void Write7SegWave(uint8_t value);
void firstADC_Read(void);
void getXY_Touchpad(void);
void start_Other_IC_Communication(void);
void ADC_Read(void);
float shaper(float);
double clipMe(double n, double lower, double upper);


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

uint16_t mess = 0;
	static HAL_DMA_StateTypeDef spi1tx;
	static HAL_DMA_StateTypeDef spi1rx;

void StartAudio(void)
{ 

	int16_t dummy1 = 0;
	uint16_t dummy2 = 0;	

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high to make sure it's not selected
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Other IC CS pin goes high to stop conversion
	HAL_Delay(100);
	//HAL_Delay(100);
	//HAL_Delay(100);
	//HAL_Delay(100);
	//now to send all the necessary messages to the codec
	AudioCodec_init();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	
	//MX_SPI1_Init();
	//firstXY_Touchpad();
	//start_Other_IC_Communication();
	
	//HAL_Delay(100);
	//set up the scalars that set the decay times for the envelopes
  //firstADC_Read();
	initScalars();
	// time to set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	myStatus = HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
  
	readAndWriteOtherChip(dummy1, dummy2);
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
	//ADC_Read();

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
		
		
			//ADC_Read();
		//}
	}
	/*
	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
		{
			getXY_Touchpad();
			//Write7SegWave((myTouchpad[0] / 4));
			ADC_Read();
		}
	}
	*/
	//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)IC_tx, (uint8_t *)IC_rx, IC_BUFFER_SIZE);
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
		temp = phasor * VECTOR_LENGTH;
		intPart = temp;
		fracPart = temp - intPart;
		samp0 = theFunction[intPart];
		if (++intPart >= VECTOR_LENGTH)
			intPart = 0;
		samp1 = theFunction[intPart];
		single_samp = (samp0 + (samp1 - samp0) * fracPart) * INV_TWO_TO_15; //scale amplitude down

		updatePhase();
		return single_samp;
	//}
}

float testTemp;
int whichknob = 0;

//uint16_t OtherICBufferIndexNum = 0;
void fillBufferWithInputProcess(uint8_t buffer_offset)
{
			uint16_t i = 0;
			int16_t current_sample = 0;   
	   
	
			if(buffer_offset == 0)
      {
					for (i = 0; i < (HALF_BUFFER_SIZE); i++)
					{
						if ((i & 1) == 0)
						{
							//testTemp = myProcess(Receive_Buffer[i] * INV_TWO_TO_15) * TWO_TO_15;
							//int16_t OtherChip = readAndWriteOtherChip(Receive_Buffer[i], OtherICBufferIndexNum);
							current_sample = testTemp;
							//OtherICBufferIndexNum++;
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
 							//testTemp = myProcess((float) (Receive_Buffer[HALF_BUFFER_SIZE + i] * INV_TWO_TO_15)) * TWO_TO_15;
							//int16_t OtherChip = readAndWriteOtherChip(Receive_Buffer[HALF_BUFFER_SIZE + i], OtherICBufferIndexNum);
							current_sample = testTemp;
							//OtherICBufferIndexNum++;
						}
						Send_Buffer[HALF_BUFFER_SIZE + i] = current_sample;
					}
      } 
			
			/*
			readExternalADC(whichknob);
			whichknob++;
			if (whichknob > 6)
			{
				whichknob = 0;
			}
			*/
}

float myProcess(float AudioIn)
{
	float sample;
  float envGain[2];

	
	scalar[0] =( powf( 0.5f, (1.0f/((((float)ADC_values[4]) * .5f)* INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	scalar[1] =( powf( 0.5f, (1.0f/((((float)ADC_values[5]) * .25f) * INV_TWO_TO_12 * (float)SAMPLE_RATE))));
	
	//set frequency of sine and delay
	phaseInc = (MtoF((currParamValue[ADC_FREQ]) * 109.0f + 25.f)) * INV_SAMPLE_RATE;
	setDelay(currParamValue[ADC_DELAY]);
	
	AudioGateVal = ((float)ADC_values[3]) * INV_TWO_TO_12 * 0.2f;
	env_detector_thresh = AudioGateVal;
	if (AudioIn < AudioGateVal)
	{
		AudioIn = 0.f;
	}
	//float OtherChip = readAndWriteOtherChip(AudioIn, 0);
	envGain[0] = adc_env_detector(AudioIn, 0);
  envGain[1] = adc_env_detector(AudioIn, 1);
	
	sample = ((KSprocess(AudioIn) * 0.55f) + (AudioIn * 0.6f));
	sample += (0.33f * ((wavetableSynth() * envGain[0]) + (((pinkNoise() * 1.0f) + whiteNoise() * .13f)* envGain[1])));
	//sample += (0.8f * wavetableSynth());
	//sample = pinkNoise() * 0.1f;
  sample = highpass3(FastTanh2Like4Term(sample * gainBoost));
	//sample = highpass(shaper(sample));

	//update Paramesters

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
	if (m >= NUM_PARAMS)
	{
		m = 0;
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


void getADC_Raw(uint8_t myKnob)
{
	if (myKnob == 2)
	{

		newFeedback = interpolateFeedback(ADC_values[2]);
		if (newFeedback != destParamValue[ADC_FEEDBACK])
		{
			setRamp(ADC_FEEDBACK, currParamValue[ADC_FEEDBACK], newFeedback, 10.f);
		}
	}
	if (myKnob == 1)
	{

		newFreq =  (ADC_values[1] * INV_TWO_TO_12) * (ADC_values[0] * INV_TWO_TO_12); 
		//newFreq = (myTouchpad[0] * INV_TWO_TO_8);
		if (newFreq != destParamValue[ADC_FREQ])
		{
			setRamp(ADC_FREQ, currParamValue[ADC_FREQ], newFreq, 70.f);
		}
	}
	if (myKnob == 0)
	{

		newDelay = interpolateDelay((4096 - ADC_values[0]));
		//newDelay = interpolateDelay((4096 - (myTouchpad[0] << 4)));
		if (newDelay != destParamValue[ADC_DELAY])
		{
			setRamp(ADC_DELAY, currParamValue[ADC_DELAY], newDelay, 70.f);
		}
	}
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
	//mParamInc[param_index] = (new_value - old_value)/ms_ramp * INV_SR_MS;
	mParamInc[param_index] = (new_value - old_value)/ms_ramp * INV_SR_DIV_PARAMS_MS;
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

float xs3 = 0.0f;
float ys3 = 0.0f;
float R3 = (1.f-(126.f/SAMPLE_RATE));
float y3 = 0.0f;

float highpass3(float x3)
{
	
y3 = x3 - xs3 + R3 * ys3;
	ys3 = y3;
	xs3 = x3;
	return y3;
}

float gate(float input)
{
	if (input > gateGain)
	{
		return input;
	}
	else return 0.0f;
}

double clipMe(double n, double lower, double upper)
{
		return fmax(lower, fmin(n, upper));
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
	if (scaled_raw < 0.333333f)
	{
		return (DelayLookup[0] + ((DelayLookup[1] - DelayLookup[0]) * (scaled_raw * 3.f)));
	}
	else if (scaled_raw < 0.75f)
	{
		return (DelayLookup[1] + ((DelayLookup[2] - DelayLookup[1]) * ((scaled_raw - 0.33333f) * 2.4f)));
	}
	else
	{
		return (DelayLookup[2] + ((DelayLookup[3] - DelayLookup[2]) * ((scaled_raw - 0.75f) * 1.333f)));
	}	
}

float interpolateFeedback(float raw_data)
{
	float scaled_raw = raw_data * INV_TWO_TO_12;
	float delaytime = ADC_values[0] * INV_TWO_TO_12;
	if (delaytime < 0.7f)
	{
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
	else if (delaytime < 0.85f)
	{
		if (scaled_raw < 0.2f)
		{
			return (FeedbackLookup2[0] + ((FeedbackLookup2[1] - FeedbackLookup2[0]) * ((scaled_raw) * 5.0f)));
		}		
		else if (scaled_raw < 0.6f)
		{
			return (FeedbackLookup2[1] + ((FeedbackLookup2[2] - FeedbackLookup2[1]) * ((scaled_raw - 0.2f) * 2.5f)));
		}		
		if (scaled_raw < .95f)
		{
			return (FeedbackLookup2[2] + ((FeedbackLookup2[3] - FeedbackLookup2[2]) * ((scaled_raw - 0.6f) * 2.857142857142f)));
		}
		else
		{
			return (FeedbackLookup2[3] + ((FeedbackLookup2[4] - FeedbackLookup2[3]) * ((scaled_raw - 0.95f) * 20.0f )));
		}
	}
		
	else if (delaytime < 0.95f)
	{
		if (scaled_raw < 0.2f)
		{
			return (FeedbackLookup3[0] + ((FeedbackLookup3[1] - FeedbackLookup3[0]) * ((scaled_raw) * 5.0f)));
		}		
		else if (scaled_raw < 0.6f)
		{
			return (FeedbackLookup3[1] + ((FeedbackLookup3[2] - FeedbackLookup3[1]) * ((scaled_raw - 0.2f) * 2.5f)));
		}		
		if (scaled_raw < .95f)
		{
			return (FeedbackLookup3[2] + ((FeedbackLookup3[3] - FeedbackLookup3[2]) * ((scaled_raw - 0.6f) * 2.857142857142f)));
		}
		else
		{
			return (FeedbackLookup3[3] + ((FeedbackLookup3[4] - FeedbackLookup3[3]) * ((scaled_raw - 0.95f) * 20.0f )));
		}
	}		
	else
	{
		if (scaled_raw < 0.2f)
		{
			return (FeedbackLookup4[0] + ((FeedbackLookup4[1] - FeedbackLookup4[0]) * ((scaled_raw) * 5.0f)));
		}		
		else if (scaled_raw < 0.6f)
		{
			return (FeedbackLookup4[1] + ((FeedbackLookup4[2] - FeedbackLookup4[1]) * ((scaled_raw - 0.2f) * 2.5f)));
		}		
		if (scaled_raw < .95f)
		{
			return (FeedbackLookup4[2] + ((FeedbackLookup4[3] - FeedbackLookup4[2]) * ((scaled_raw - 0.6f) * 2.857142857142f)));
		}
		else
		{
			return (FeedbackLookup4[3] + ((FeedbackLookup4[4] - FeedbackLookup4[3]) * ((scaled_raw - 0.95f) * 20.0f )));
		}
	}
}

void readExternalADC(int which)
{
	uint16_t knobsendbyte;
	uint8_t ADC_out[2];		
	uint8_t knobnum;
	uint16_t knobval12bit;

	//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
	knobsendbyte = 6208 + (which << 7);
	ADC_out[0] = knobsendbyte >> 8;
	ADC_out[1] = knobsendbyte & 255;
	
	// write to ADC chip (asking for data on next channel)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ADC CS pin goes low to initiate conversion

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)ADC_out, (uint8_t *)ADC_in, ADC_BUFFERSIZE, TIMEOUT);
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
		feedbacksamp = clipMe(highpass(m_output0), -.9999f, .9999f);
		
		return FastTanh2Like4Term(highpass2(feedbacksamp));
}

float shaper(float input)
{
	float shaperOut = 0.f;
	// shaper
	float fx = input * 2.f;	// prescale
	double w, c, xc, xc2, xc4;
	const double sqrt8 = 2.82842712475f;
	const double wscale = 1.30612244898f; // 1/w(1).
	
	xc = clipMe(fx, -sqrt8, sqrt8);
	xc2 = xc*xc;
	c = 0.5f*fx*(3.f - (xc2));
	xc4 = xc2 * xc2;
	w = (1.f - xc2*0.25f + xc4*0.015625f) * wscale;
	shaperOut = w*(c+ 0.05f*xc2)*(m_drive + 0.75f);
	shaperOut *= 0.5f;	// post_scale
	return shaperOut;
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
	
	//fillBufferWithInputProcess(1);
	//myStatus = HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  //pullUpOtherChipCS();	  
		
	//fillBufferWithInputProcess(0);
	//myStatus = HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t*)&Send_Buffer[0], (uint16_t*)&Receive_Buffer[0], AUDIO_BUFFER_SIZE);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	;
}

uint8_t whichKnob = 0;

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
		ADC_Read();
	}
	/*
	//if it's SPI2, that's the ADC ready to scan the next knob.
	if(hspi == &hspi2)
	{
	  uint16_t knobsendbyte;	
		uint8_t knobnum;
		uint16_t knobval12bit;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high
		//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
		knobsendbyte = 6208 + (whichKnob << 7);
		ADC_out[0] = knobsendbyte >> 8;
		ADC_out[1] = knobsendbyte & 255;
		knobnum = ADC_in[0] >> 4;
		knobval12bit = (ADC_in[1] | ((ADC_in[0] & 15) << 8));
		ADC_values[knobnum] = knobval12bit;
		whichKnob++;
		if (whichKnob > 6)
		{
			whichKnob = 0;
		}
		if (whichKnob < 3)
		{
			getADC_Raw(whichKnob);
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ADC CS pin goes low to initiate conversion
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)ADC_out, (uint8_t *)ADC_in, ADC_BUFFERSIZE, IC_TIMEOUT);
	}
	*/
}


void ADC_Read(void)
{
	  uint16_t knobsendbyte;	
		static uint8_t knobnum;
		uint16_t knobval12bit;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // ADC CS pin goes high
		//create the word to send to the knob ADC chips (telling the ADC which channel to scan this time around)
		knobsendbyte = 6208 + (whichKnob << 7);
		ADC_out[0] = knobsendbyte >> 8;
		ADC_out[1] = knobsendbyte & 255;
		knobnum = ADC_in[0] >> 4;
		knobval12bit = (ADC_in[1] | ((ADC_in[0] & 15) << 8));
		ADC_values[knobnum] = knobval12bit;
	  Write7SegWave((uint8_t)(ADC_values[0] >> 5));
		whichKnob++;
		if (whichKnob > 5)
		{
			whichKnob = 0;
		}
		if (whichKnob < 3)
		{
			getADC_Raw(whichKnob); //apply curve functions if they are the first 3 knobs
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ADC CS pin goes low to initiate conversion
		//HAL_Delay(1);
		/*
		while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
		{
			;
		}
		*/
		HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)ADC_out, (uint8_t *)ADC_in, ADC_BUFFERSIZE);
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

