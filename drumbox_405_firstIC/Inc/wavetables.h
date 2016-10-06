/* sine wave table ripped from http://aquaticus.info/pwm-sine-wave */
#ifndef WAVETABLES_H
#define WAVETABLES_H

#include "main.h"

#define SINE_TABLE_SIZE 2048
#define SHAPER1_TABLE_SIZE 65535
#define TANH1_TABLE_SIZE 65535
#define ADC1_TABLE_SIZE 4096
#define MTOF1_TABLE_SIZE 4096
#define FB_TABLE_SIZE 4096
#define FILTERTAN_TABLE_SIZE 4096

extern const float FB1[FB_TABLE_SIZE];
extern const float FB2[FB_TABLE_SIZE];
extern const float FB3[FB_TABLE_SIZE];
extern const float FB4[FB_TABLE_SIZE];
extern const float FB5[FB_TABLE_SIZE];
extern const float FB6[FB_TABLE_SIZE];
extern const float FB7[FB_TABLE_SIZE];
extern const float FB8[FB_TABLE_SIZE];

// mtof lookup table based on input range [0.0,1.0) in 4096 increments - midi frequency values scaled between m25 and m134 (as done in previous code)
extern const float filtertan[FILTERTAN_TABLE_SIZE];
extern const float mtof1[MTOF1_TABLE_SIZE];
extern const float adc1[ADC1_TABLE_SIZE];
extern const float shaper1[SHAPER1_TABLE_SIZE];
extern const float tanh1[TANH1_TABLE_SIZE];

extern const float sinewave[SINE_TABLE_SIZE];

#endif 
