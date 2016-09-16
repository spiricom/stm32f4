
#include "stm32f4xx_hal.h"

	 
#define NUMCHANNELS 10

extern uint16_t ADC_values[NUMCHANNELS];
extern uint16_t ADC_LastRead[NUMCHANNELS];
