#include "stdint.h"

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
