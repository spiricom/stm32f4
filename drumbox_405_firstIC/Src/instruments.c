#include "instruments.h"
#include "math.h"

#define USE_STICK 0

float clipI(float min, float val, float max) {
	
	if (val < min) {
		return min;
	} else if (val > max) {
		return max;
	} else {
		return val;
	}
}

int t808CowbellOn(t808Cowbell *cowbell, float vel) {
	
	envOn(cowbell->envGain,vel);
#if USE_STICK
	envOn(cowbell->envStick,vel);
#endif
	
	return 0;
}

float t808CowbellTick(t808Cowbell *cowbell) {
	
	float sample = 0.0f;
	
	// Mix oscillators.
	sample = (cowbell->oscMix * tick0(cowbell->p1)) + ((1.0f-cowbell->oscMix) * tick0(cowbell->p2));
	
	// Filter dive and filter.
	setFreqFromKnob(cowbell->bandpassOsc, cowbell->filterCutoff + 1000.0f*tick0(cowbell->envFilter));
	sample = tick1(cowbell->bandpassOsc,sample);
	
	sample *= (0.9f * tick0(cowbell->envGain));
	
#if USE_STICK	
	sample += (0.1f * tick0(cowbell->envStick) * tick1(cowbell->bandpassStick, tick0(cowbell->stick)));
#endif
	
	sample = tick1(cowbell->highpass, sample);
	
	return sample;
}

static int t808CowbellSetDecay(t808Cowbell *cowbell, float decay) {
	
	setEnvelopeDecay(cowbell->envGain,decay);
	//setEnvelopeDecay(cowbell->envFilter,decay);
	
	return 0;	
}

static int t808CowbellSetHighpassFreq(t808Cowbell *cowbell, float freq) {
	
	setFreq(cowbell->highpass,freq);
	
	return 0;	
}

static int t808CowbellSetBandpassFreq(t808Cowbell *cowbell, float freq) {

	cowbell->filterCutoff = freq;
	
	return 0;	
}

static int t808CowbellSetFreq(t808Cowbell *cowbell, float freq) {
	
	setFreq(cowbell->p1,freq);
	setFreq(cowbell->p2,1.48148f*freq);
	
	return 0;	
}

static int t808CowbellSetOscMix(t808Cowbell *cowbell, float oscMix) {
	
	cowbell->oscMix = oscMix;
	
	return 0;	
}

int t808CowbellInit(t808Cowbell *cowbell, float sr, float (*randomNumberGenerator)(),const float *exp_decay, const float *attack_decay_inc) {
	
	tPulseInit(&cowbell->p1,sr,0.5f);
	tPulseInit(&cowbell->p2,sr,0.5f);
	
	setFreq(cowbell->p1, 540.0f);
	setFreq(cowbell->p2, 1.48148f * 540.0f);
	
	cowbell->oscMix = 0.5f;

	tSVFInit(&cowbell->bandpassOsc, sr, SVFTypeBandpass, 2500, 1.0f);
	
	tSVFInit(&cowbell->bandpassStick, sr, SVFTypeBandpass, 1800, 1.0f);
	
	tEnvelopeInit(&cowbell->envGain, sr, 5.0f, 100.0f, 0, exp_decay, attack_decay_inc);

	tEnvelopeInit(&cowbell->envFilter, sr, 5.0, 100.0f, 0, exp_decay,attack_decay_inc);
	
	tHighpassInit(&cowbell->highpass,sr,1000.0f);
	
#if USE_STICK	
	tNoiseInit(&cowbell->stick, sr, randomNumberGenerator, NoiseTypeWhite);
	tEnvelopeInit(&cowbell->envStick, sr, 5.0f, 5.0f, 0, exp_decay, attack_decay_inc);
#endif	
	
	cowbell->tick = &t808CowbellTick;
	cowbell->on = &t808CowbellOn;
	cowbell->setOscFreq = &t808CowbellSetFreq;
	cowbell->setOscMix = &t808CowbellSetOscMix;
	cowbell->setDecay = &t808CowbellSetDecay;
	cowbell->setHighpassFreq = &t808CowbellSetHighpassFreq;
	cowbell->setOscBandpassFreq = &t808CowbellSetBandpassFreq;
	
	return 0;
}
	
static int t808HihatOn(t808Hihat *hihat, float vel) {
	
	envOn(hihat->envGain, vel);
	envOn(hihat->envStick, vel);
	
	return 0;	
}

static int t808HihatSetOscNoiseMix(t808Hihat *hihat, float oscNoiseMix) {
	
	hihat->oscNoiseMix = oscNoiseMix;
	
	return 0;	
}

static float t808HihatTick(t808Hihat *hihat) {
	
	float sample = 0.0f;
	float gainScale = 0.1666f;
	
	sample = (gainScale * tick0(hihat->p1));
	sample += (gainScale * tick0(hihat->p2));
	sample += (gainScale * tick0(hihat->p3));
	sample += (gainScale * tick0(hihat->p4));
	sample += (gainScale * tick0(hihat->p5));
	sample += (gainScale * tick0(hihat->p6));
	sample = (hihat->oscNoiseMix * sample) + ((1.0f-hihat->oscNoiseMix) * (0.8f*tick0(hihat->n)));
	
	sample = tick1(hihat->bandpassOsc, sample);
	
	sample *= tick0(hihat->envGain);
	
	sample = 0.85f * clipI(0.0f, tick1(hihat->highpass, sample), 1.0f);
	
	sample += 0.15f * tick0(hihat->envStick) * tick1(hihat->bandpassStick,tick0(hihat->stick));
	
	return sample;
}

static int t808HihatSetDecay(t808Hihat *hihat, float decay) {
	setEnvelopeDecay(hihat->envGain,decay);
	
	return 0;
}

static int t808HihatSetHighpassFreq(t808Hihat *hihat, float freq) {

	setFreq(hihat->highpass,freq);
	
	return 0;
}

static int t808HihatSetOscBandpassFreq(t808Hihat *hihat, float freq) {
	setFreqFromKnob(hihat->bandpassOsc,freq);
	
	return 0;
}


static int t808HihatSetOscFreq(t808Hihat *hihat, float freq) {
	
	setFreq(hihat->p1, 2.0f * freq);
	setFreq(hihat->p2, 3.00f * freq);
	setFreq(hihat->p3, 4.16f * freq);
	setFreq(hihat->p4, 5.43f * freq);
	setFreq(hihat->p5, 6.79f * freq);
	setFreq(hihat->p6, 8.21f * freq);
	
	return 0;
}

int t808HihatInit(t808Hihat *hihat, float sr, float (*randomNumberGenerator)(), const float *exp_decay, const float *attack_decay_inc) {
	

	tPulseInit(&hihat->p1,sr,0.5f);
	tPulseInit(&hihat->p2,sr,0.5f);
	tPulseInit(&hihat->p3,sr,0.5f);
	tPulseInit(&hihat->p4,sr,0.5f);
	tPulseInit(&hihat->p5,sr,0.5f);
	tPulseInit(&hihat->p6,sr,0.5f);
	
	/*
	tSawtoothInit(&hihat->p1,sr);
	tSawtoothInit(&hihat->p2,sr);
	tSawtoothInit(&hihat->p3,sr);
	tSawtoothInit(&hihat->p4,sr);
	tSawtoothInit(&hihat->p5,sr);
	tSawtoothInit(&hihat->p6,sr);
	*/
	
	tNoiseInit(&hihat->stick,sr,randomNumberGenerator,NoiseTypeWhite);
	tNoiseInit(&hihat->n, sr,randomNumberGenerator,NoiseTypeWhite);
	
	// need to fix SVF to be generic
	tSVFInit(&hihat->bandpassStick,sr,SVFTypeBandpass,2500.0,1.5f);
	tSVFInit(&hihat->bandpassOsc,sr,SVFTypeBandpass,3500,0.5f);
	
	tEnvelopeInit(&hihat->envGain, sr, 5.0f, 50.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&hihat->envStick, sr, 5.0f, 15.0f, 0, exp_decay, attack_decay_inc);
	
	tHighpassInit(&hihat->highpass, sr, 7000.0f);
	float fund = 40.0f;	
	setFreq(hihat->p1, 2.0f * fund);
	setFreq(hihat->p2, 3.00f * fund);
	setFreq(hihat->p3, 4.16f * fund);
	setFreq(hihat->p4, 5.43f * fund);
	setFreq(hihat->p5, 6.79f * fund);
	setFreq(hihat->p6, 8.21f * fund);
	
	hihat->setDecay = t808HihatSetDecay;
	hihat->setHighpassFreq = t808HihatSetHighpassFreq;
	hihat->setOscFreq = &t808HihatSetOscFreq;
	hihat->setOscBandpassFreq = &t808HihatSetOscBandpassFreq;
	hihat->setOscNoiseMix = &t808HihatSetOscNoiseMix;
	hihat->on = &t808HihatOn;
	hihat->tick = &t808HihatTick;
	
	return 0; 
}


static int t808SnareOn(t808Snare *snare, float vel) {
	
	envOn(snare->tone1EnvOsc, vel);
	envOn(snare->tone1EnvGain, vel);
	envOn(snare->tone1EnvFilter, vel);
			
	envOn(snare->tone2EnvOsc, vel);
	envOn(snare->tone2EnvGain, vel);
	envOn(snare->tone2EnvFilter, vel);
			
	envOn(snare->noiseEnvGain, vel);
	envOn(snare->noiseEnvFilter, vel);

	return 0;	
}

static int t808SnareSetTone1Freq(t808Snare *snare, float freq) {
	snare->tone1Freq = freq;
	setFreq(snare->tone1Osc,freq);
	
	return 0;
}

static int t808SnareSetTone2Freq(t808Snare *snare, float freq) {
	snare->tone2Freq = freq;
	setFreq(snare->tone2Osc,freq);
	
	return 0;
}

static int t808SnareSetTone1Decay(t808Snare *snare, float decay) {
	setEnvelopeDecay(snare->tone1EnvGain,decay);
	
	return 0;
}

static int t808SnareSetTone2Decay(t808Snare *snare, float decay) {
	setEnvelopeDecay(snare->tone2EnvGain,decay);
	
	return 0;
}

static int t808SnareSetNoiseDecay(t808Snare *snare, float decay) {
	setEnvelopeDecay(snare->noiseEnvGain,decay);
	
	return 0;
}

static int t808SnareSetToneNoiseMix(t808Snare *snare, float toneNoiseMix) {
	
	snare->toneNoiseMix = toneNoiseMix;
	
	return 0;
}

static int t808SnareSetNoiseFilterFreq(t808Snare *snare, float noiseFilterFreq) {
	
	snare->noiseFilterFreq = noiseFilterFreq;
	
	return 0;
}

static int t808SnareSetNoiseFilterQ(t808Snare *snare, float noiseFilterQ) {
	
	setQ(snare->noiseLowpass, noiseFilterQ);
	
	return 0;
}


static float t808SnareTick(t808Snare *snare) {
	
	setFreq(snare->tone1Osc, snare->tone1Freq + (50.0f * tick0(snare->tone1EnvOsc)));
	float tone1 = tick0(snare->tone1Osc);
	
	setFreqFromKnob(snare->tone1Lowpass, 2000 + (500 * tick0(snare->tone1EnvFilter)));
	tone1 = tick1(snare->tone1Lowpass, tone1) * tick0(snare->tone1EnvGain);
	
	setFreq(snare->tone2Osc, snare->tone2Freq + (50.0f * tick0(snare->tone2EnvOsc)));
	float tone2 = snare->tone2Osc.tick(&snare->tone2Osc);
	
	setFreqFromKnob(snare->tone2Lowpass, snare->noiseFilterFreq + (500 * tick0(snare->tone2EnvFilter)));
	tone2 = tick1(snare->tone2Lowpass, tone2) * tick0(snare->tone2EnvGain);
	
	float noise = tick0(snare->noiseOsc);
	setFreqFromKnob(snare->noiseLowpass, snare->noiseFilterFreq +(500 * tick0(snare->noiseEnvFilter)));
	noise = tick1(snare->noiseLowpass, noise) * tick0(snare->noiseEnvGain);
	
	float sample = (snare->toneNoiseMix)*(tone1 * snare->tone1Gain + tone2 * snare->tone2Gain) + (1.0f-snare->toneNoiseMix) * (noise * snare->noiseGain);

	return sample;
}

int t808SnareInit(t808Snare *snare, float sr, float (*randomNumberGenerator)(), const float *exp_decay, const float *attack_decay_inc) {
	
	tTriangleInit(&snare->tone1Osc, sr);
	tSVFInit(&snare->tone1Lowpass, sr, SVFTypeLowpass, 2000, 1.0f);
	tEnvelopeInit(&snare->tone1EnvOsc, sr, 3.0f, 20.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&snare->tone1EnvGain, sr, 10.0f, 200.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&snare->tone1EnvFilter, sr, 3.0f, 200.0f, 0, exp_decay, attack_decay_inc);
	snare->tone1Gain = 0.5f;
	
	tTriangleInit(&snare->tone2Osc, sr);
	tSVFInit(&snare->tone2Lowpass, sr, SVFTypeLowpass, 2000, 1.0f);
	tEnvelopeInit(&snare->tone2EnvOsc, sr, 3.0f, 25.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&snare->tone2EnvGain, sr, 10.0f, 200.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&snare->tone2EnvFilter, sr, 3.0f, 200.0f, 0, exp_decay, attack_decay_inc);
	snare->tone2Gain = 0.5f;

	tNoiseInit(&snare->noiseOsc, sr, randomNumberGenerator, NoiseTypeWhite);
	tSVFInit(&snare->noiseLowpass, sr, SVFTypeLowpass, 2000, 3.0f);
	tEnvelopeInit(&snare->noiseEnvGain, sr, 10.0f, 125.0f, 0, exp_decay, attack_decay_inc);
	tEnvelopeInit(&snare->noiseEnvFilter, sr, 3.0f, 100.0f, 0, exp_decay, attack_decay_inc);
	snare->noiseGain = 0.25f;
	
	snare->setTone1Freq = &t808SnareSetTone1Freq;
	snare->setTone2Freq = &t808SnareSetTone2Freq;
	snare->setTone1Decay = &t808SnareSetTone1Decay;
	snare->setTone2Decay = &t808SnareSetTone2Decay;
	snare->setNoiseDecay = &t808SnareSetNoiseDecay;
	snare->setToneNoiseMix = &t808SnareSetToneNoiseMix;
	snare->setNoiseFilterFreq = &t808SnareSetNoiseFilterFreq;
	snare->setNoiseFilterQ = &t808SnareSetNoiseFilterQ;
	snare->on = &t808SnareOn;
	snare->tick = &t808SnareTick;
	
	return 0; 
}
