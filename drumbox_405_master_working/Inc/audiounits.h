#include "stdint.h"

/* Phasor [0.0, 1.0) */
typedef struct _tPhasor {
	float phase; 
	float inc;
	float inv_sr; 
	int(*freq)(struct _tPhasor *self, float freq);
	float(*step)(struct _tPhasor *self);
} tPhasor; 

int tPhasorInit(tPhasor *p, float sr);

/* Cycle */
typedef struct _tCycle { 
	// Underlying phasor
	float phase;
	float inc;
	float inv_sr;
	
	// Wavetable synthesis
	const float *wt; //wavetable
	int wtlen; //wavetable length
	int(*freq)(struct _tCycle *self, float freq);
	float(*step)(struct _tCycle *self);
} tCycle;

int tCycleInit(tCycle *c, float sr, const float *table, int len);

/* Sawtooth */
typedef struct _tSawtooth{ 
	// Underlying phasor
	float phase;
	float inc;
	float inv_sr;
	
	int(*freq)(struct _tSawtooth *self, float freq);
	float(*step)(struct _tSawtooth *self);
} tSawtooth;

int tSawtoothInit(tSawtooth *t, float sr);

/* Triangle */
typedef struct _tTriangle{ 
	// Underlying phasor
	float phase;
	float inc;
	float inv_sr;
	
	int(*freq)(struct _tTriangle *self, float freq);
	float(*step)(struct _tTriangle *self);
} tTriangle;

int tTriangleInit(tTriangle *t, float sr);

/* Pulse */
typedef struct _tPulse{ 
	// Underlying phasor
	float phase;
	float inc;
	float inv_sr;
	
	float pw; 
	int(*pwidth)(struct _tPulse *self, float pwidth);
	int(*freq)(struct _tPulse *self, float freq);
	float(*step)(struct _tPulse *self);
} tPulse;

int tPulseInit(tPulse *t, float sr, float pwidth);

/* Noise */
typedef enum NoiseType {
	NoiseTypeWhite=0,
	NoiseTypePink,
	NoiseTypeNil,
} NoiseType;

typedef struct _tNoise { 
	float pinkb0, pinkb1, pinkb2;
	float(*rand)();
	float(*step)(struct _tNoise *self);
} tNoise;

int tNoiseInit(tNoise *c, float sr, float (*randomNumberGenerator)(), NoiseType type);

