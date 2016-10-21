#ifndef UTILITIES_H
#define UTILITIES_H

#include "stdint.h"

#define setRate(THIS,RATE)			THIS.setRate(&THIS,RATE)

typedef enum ControlParameterType {
	ControlParameterFeedback = 0,
	ControlParameterDive,
	ControlParameterSineDecay,
	ControlParameterNoiseWidth,
	ControlParameterNoiseDecay,
	ControlParameterNoiseCutoff,
	ControlParameterNil,
} ControlParameterType;

typedef enum ControlParamaterXYType {
	ControlParameterBR = 0,
	ControlParameterBL,
	ControlParameterMR,
	ControlParameterML,
	ControlParameterTR,
	ControlParameterTL,
	ControlParameterXYNil,
} ControlParameterXYType;

typedef enum SmoothedParameterType {
	SmoothedParameterDelay= 0,
	SmoothedParameterSineFreq,
	SmoothedParameterFeedback,
	SmoothedParameterNil,
} SmoothedParameterType;

typedef struct _tMetro {
	
	uint32_t peak,phase; 
	float sr; 
	int(*setRate)(struct _tMetro *self, float rate);
	int(*tick)(struct _tMetro *self);
	
} tMetro;

int tMetroInit(tMetro *m, float sr, float rate);

/* Ramp */
typedef struct _tRamp {
	float inc;
	float inv_sr_ms;
	float curr,dest;
	float time;
	int samples_per_tick;
	int(*setTime)(struct _tRamp *self, float time);
	int(*setDest)(struct _tRamp *self, float dest);
	float(*tick)(struct _tRamp *self);
} tRamp; 

int tRampInit(tRamp *r, float sr, float time, int samples_per_tick);

#endif
