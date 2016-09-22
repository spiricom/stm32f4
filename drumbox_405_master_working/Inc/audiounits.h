#include "stdint.h"

#define VERY_SMALL_FLOAT 1.0e-38f

/* Envelope Follower */
typedef struct _tEnvelopeFollower {
	float y, a_thresh, d_coeff; 
	int(*attackThresh)(struct _tEnvelopeFollower *self, float attackThresh);
	int(*decayCoeff)(struct _tEnvelopeFollower *self, float decayCoeff);
	float(*tick)(struct _tEnvelopeFollower *self, float x);
} tEnvelopeFollower; 

int tEnvelopeFollowerInit(tEnvelopeFollower *ef, float attackThreshold, float decayCoeff);

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

typedef struct _tHighpass {
	float inv_sr;
	float xs, ys, R;
	float cutoff;
	int(*freq)(struct _tHighpass *self, float x);
	float(*tick)(struct _tHighpass *self, float x);
} tHighpass;

int tHighpassInit(tHighpass *hp, float sr, float freq);

