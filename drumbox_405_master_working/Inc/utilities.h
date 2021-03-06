#include "stdint.h"

typedef enum ControlParameterType {
	ControlParameterDelay = 0,
	ControlParameterFrequency,
	ControlParameterFeedback,
	ControlParameterThreshold,
	ControlParameterSineDecay,
	ControlParameterNoiseDecay,
	ControlParamaterNil,
} ControlParamaterType;

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
