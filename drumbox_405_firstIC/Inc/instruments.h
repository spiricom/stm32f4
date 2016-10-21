#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H

#include "audiounits.h"

#define setCowbellBandpassFreqFromKnob(THIS,FREQ)		THIS.setOscBandpassFreq(&THIS,FREQ)
#define setCowbellOscMix(THIS,MIX)									THIS.setOscMix(&THIS,MIX)
#define setCowbellFreq(THIS,FREQ)										THIS.setOscFreq(&THIS,FREQ)
#define setCowbellDecay(THIS,DECAY)									THIS.setDecay(&THIS,DECAY)

#define setHihatDecay(THIS,DECAY)										THIS.setDecay(&THIS,DECAY)
#define setHihatHighpassFreq(THIS,FREQ)							THIS.setHighpassFreq(&THIS,FREQ)
#define setHihatFreq(THIS,FREQ)											THIS.setOscFreq(&THIS,FREQ)
#define setHihatOscBandpassFreq(THIS,FREQ) 					THIS.setOscBandpassFreq(&THIS,FREQ)
#define setHihatOscNoiseMix(THIS,FREQ)							THIS.setOscNoiseMix(&THIS,FREQ)

#define setSnareTone1Freq(THIS,FREQ)								THIS.setTone1Freq(&THIS,FREQ)
#define setSnareTone2Freq(THIS,FREQ)								THIS.setTone2Freq(&THIS,FREQ)
#define setSnareTone1Decay(THIS,DECAY)						  THIS.setTone1Decay(&THIS,DECAY)
#define setSnareTone2Decay(THIS,DECAY)							THIS.setTone2Decay(&THIS,DECAY)
#define setSnareNoiseDecay(THIS,DECAY)						  THIS.setNoiseDecay(&THIS,DECAY)
#define setSnareToneNoiseMix(THIS,MIX)							THIS.setToneNoiseMix(&THIS,MIX)
#define setSnareNoiseFilterFreq(THIS,FREQ)					THIS.setNoiseFilterFreq(&THIS,FREQ)
#define setSnareNoiseFilterQ(THIS,Q)								THIS.setNoiseFilterQ(&THIS,Q)

typedef struct _t808Cowbell {
	
	tPulse p1,p2;
	tNoise stick;
	tSVF bandpassOsc, bandpassStick;
	tEnvelope envGain, envStick, envFilter; 
	float oscMix;
	float filterCutoff;
	
	int(*setOscBandpassFreq)(struct _t808Cowbell *self, float freq);
	int(*setOscMix)(struct _t808Cowbell *self, float oscMix);
	int(*setOscFreq)(struct _t808Cowbell *self, float freq);
	int(*setDecay)(struct _t808Cowbell *self, float decay);
	
	int(*on)(struct _t808Cowbell *self, float vel);
	float(*tick)(struct _t808Cowbell *self);
	
} t808Cowbell;

int t808CowbellInit(t808Cowbell *cowbell, float sr, float (*randomNumberGenerator)(),const float *exp_decay, const float *attack_decay_inc);

typedef struct _t808Hihat {

	// 6 Square waves
	tPulse p1,p2,p3,p4,p5,p6;
	tNoise n;
	tSVF bandpassOsc,bandpassStick;
	tEnvelope envGain,envStick;
	tHighpass highpass;
	tNoise stick;
	
	float oscNoiseMix;
	
	int(*setHighpassFreq)(struct _t808Hihat *self, float freq);
	int(*setDecay)(struct _t808Hihat *self, float decay);
	int(*setOscFreq)(struct _t808Hihat *self, float freq);
	int(*setOscBandpassFreq)(struct _t808Hihat *self, float freq);
	int(*setOscNoiseMix)(struct _t808Hihat *self, float oscNoiseMix);
	float(*tick)(struct _t808Hihat *self);
	int(*on)(struct _t808Hihat *self, float vel);
	
} t808Hihat; 

int t808HihatInit(t808Hihat *hihat, float sr, float (*randomNumberGenerator)(),const float *exp_decay, const float *attack_decay_inc);

typedef struct _t808Snare {

	// Tone 1, Tone 2, Noise
	tTriangle tone1Osc, tone2Osc; // Tri (not yet antialiased or wavetabled)
	tNoise noiseOsc; 
	tSVF tone1Lowpass, tone2Lowpass, noiseLowpass; // Lowpass from SVF filter
	tEnvelope tone1EnvOsc, tone2EnvOsc;
	tEnvelope tone1EnvGain, tone2EnvGain, noiseEnvGain;
	tEnvelope tone1EnvFilter, tone2EnvFilter, noiseEnvFilter;
	float tone1Gain, tone2Gain, noiseGain;
	
	float toneNoiseMix;

	
	float tone1Freq, tone2Freq;
	
	float noiseFilterFreq;

	float(*tick)(struct _t808Snare *self);
	int(*on)(struct _t808Snare *self, float vel);
	int(*setTone1Freq)(struct _t808Snare *self, float freq);
	int(*setTone2Freq)(struct _t808Snare *self, float freq);
	int(*setTone1Decay)(struct _t808Snare *self, float decay);
	int(*setTone2Decay)(struct _t808Snare *self, float decay);
	int(*setNoiseDecay)(struct _t808Snare *self, float decay);
	int(*setNoiseFilterFreq)(struct _t808Snare *self, float freq);
	int(*setNoiseFilterQ)(struct _t808Snare *self, float Q);
	int(*setToneNoiseMix)(struct _t808Snare *self, float toneNoiseMix);
	
	
} t808Snare; 

int t808SnareInit(t808Snare *snare, float sr, float (*randomNumberGenerator)(), const float *exp_decay, const float *attack_decay_inc);

#endif
