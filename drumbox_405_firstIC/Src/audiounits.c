// Audio Units
#include "audiounits.h"
#include "wavetables.h"
#include "math.h"

#define TWO_TO_16 65536.f

/* Envelope */

#define EXPONENTIAL_TABLE_SIZE 65536

float clipAU(float min, float val, float max) {
	
	if (val < min) {
		return min;
	} else if (val > max) {
		return max;
	} else {
		return val;
	}
}
// we may want inverse Attack and Decay wavetables to avoid division

static int tEnvelopeAttack(tEnvelope *env, float attack) {
	
	uint16_t attackIndex;
	
	if (attack < 0) {
		attackIndex = 0;
	} else if (attack < 8192) { 
		attackIndex = ((uint16_t)(attack * 8.0f))-1;
	} else {
		attackIndex = UINT16_MAX;
	}
	
	env->attackInc = env->inc_buff[attackIndex];
	
	return 0;
}

static int tEnvelopeDecay(tEnvelope *env, float decay) {
	
	uint16_t decayIndex;
	
	if (decay < 0) {
		decayIndex = 0;
	} else if (decay < 8192) { 
		decayIndex = ((uint16_t)(decay * 8.0f))-1;
	} else {
		decayIndex = ((uint16_t)(8192 * 8.0f))-1;
	}

	env->decayInc = env->inc_buff[decayIndex];
	
	return 0;
}

static int tEnvelopeLoop(tEnvelope *env, int loop) {
	
	env->loop = loop;
	
	return 0;
}


static int tEnvelopeOn(tEnvelope *env, float velocity) {
	env->attackPhase = 0;
	env->decayPhase = 0;
	env->inAttack = 1;
	env->inDecay = 0;
	env->gain = velocity;
	return 0;
}

static float tEnvelopeTick(tEnvelope *env) {
	
	float out;
	
	if (env->inAttack) {
		
		uint32_t intPart = (uint32_t)env->attackInc;
		float fracPart = env->attackInc - (float)intPart;
		
		// Increment envelope attack.
		env->attackPhase += intPart;
		
		
		// If attack done, time to turn around.
		if (env->attackPhase > UINT16_MAX) {
			env->inDecay = 1;
			env->inAttack = 0;
			return env->gain * 1.0f;
		} else {
			// do interpolation !
			return env->gain * (1.0f - env->exp_buff[UINT16_MAX - env->attackPhase]); // inverted and backwards to get proper rising exponential shape/perception 
		}
	
	} else if (env->inDecay) {
		
		uint32_t intPart = (uint32_t)env->decayInc;
		float fracPart = env->decayInc - (float)intPart;
		
		// Increment envelope decay;
		env->decayPhase += intPart;
		
		// If decay done, finish. 
		if (env->decayPhase >= UINT16_MAX) {
			env->inDecay = 0;
			if (env->loop) {
				env->attackPhase = 0;
				env->decayPhase = 0;
				env->inAttack = 1;
			}
			return 0.0f;
		} else {
			
			return env->gain * (env->exp_buff[env->decayPhase]); // do interpolation !
		}		
	} else {
		
		return 0.0f;
	}
	
}

// exponentialTable must be of size 65536

int tEnvelopeInit(tEnvelope *env, float sr, float attack, float decay, int loop, const float *exponentialTable, const float *attackDecayIncTable) {

	env->inv_sr = 1.0f/sr; 
	env->exp_buff = exponentialTable;
	env->inc_buff = attackDecayIncTable;
	env->buff_size = sizeof(exponentialTable);
	
	env->loop = loop;

	if (attack > 8192) 
		attack = 8192;  
	if (attack < 0) 
		attack = 0;
	
	if (decay > 8192) 
		decay = 8192;  
	if (decay < 0) 
		decay = 0;
		
	uint16_t attackIndex = ((uint16_t)(attack * 8.0f))-1; 
	uint16_t decayIndex = ((uint16_t)(decay * 8.0f))-1;
	if (attack < 0)
		attackIndex = 0;
	if (decayIndex < 0)
		decayIndex = 0;
	env->attackInc = env->inc_buff[attackIndex];
	env->decayInc = env->inc_buff[decayIndex];
	
	env->on = &tEnvelopeOn;
	env->setLoop = &tEnvelopeLoop;
	env->setAttack = &tEnvelopeAttack;
	env->setDecay = &tEnvelopeDecay;
	env->tick = &tEnvelopeTick;
	return 0; 
}

/* Phasor */
static int phasorFreq(tPhasor *p, float freq) {
	
	p->inc = freq * p->inv_sr;
	
	return 0;
}

static float phasortick(tPhasor *p) {
	
	p->phase += p->inc;
	
	if (p->phase >= 1.0f) p->phase -= 1.0f; 

	return p->phase;
}

int tPhasorInit(tPhasor *p, float sr) {

	p->phase = 0.0f;
	p->inc = 0.0f;
	p->inv_sr = 1.0f/sr;
	p->setFreq = &phasorFreq;
	p->tick = &phasortick;
	
	return 0; 
}

float tSVFTick(tSVF *svf, float v0) {
	
	float v1,v2,v3;
	float low,high;
	v3 = v0 - svf->ic2eq;
	v1 = (svf->a1 * svf->ic1eq) + (svf->a2 * v3);
	v2 = svf->ic2eq + (svf->a2 * svf->ic1eq) + (svf->a3 * v3);
	svf->ic1eq = (2.0f * v1) - svf->ic1eq;
	svf->ic2eq = (2.0f * v2) - svf->ic2eq;
	
	if (svf->type == SVFTypeLowpass) {
		
		return v2;
	} else if (svf->type == SVFTypeBandpass) {
		
		return v1;
	} else if (svf->type == SVFTypeHighpass) {
		
		return (v0 - (svf->k * v1) - v2);
	} else if (svf->type == SVFTypeNotch) {
		
		return v0 - (svf->k * v1); 
	} else if (svf->type == SVFTypePeak) {
		
		return v0 - (svf->k * v1) - 2.0f * v2;
	} else {
		
		return 0.0f;
	}
	
}

int tSVFSetFreq(tSVF *svf, uint16_t cutoffKnob) {
	
	svf->g = filtertan[cutoffKnob];
	svf->a1 = 1.0f/(1.0f + svf->g * (svf->g + svf->k));
	svf->a2 = svf->g * svf->a1;
	svf->a3 = svf->g * svf->a2;
	
	return 0;
}

int tSVFSetQ(tSVF *svf, float Q) {
	
	svf->k = 1.0f/Q;
	svf->a1 = 1.0f/(1.0f + svf->g * (svf->g + svf->k));
	svf->a2 = svf->g * svf->a1;
	svf->a3 = svf->g * svf->a2;
	
	return 0;
}

int tSVFInit(tSVF *svf, float sr, SVFType type, uint16_t cutoffKnob, float Q) {
	
	svf->inv_sr = 1.0f/sr;
	svf->type = type;
	
	svf->ic1eq = 0;
	svf->ic2eq = 0;
	
	float a1,a2,a3,g,k;
	g = filtertan[cutoffKnob]; 
	k = 1.0f/Q;
	a1 = 1.0f/(1.0f+g*(g+k));
	a2 = g*a1;
	a3 = g*a2;
	
	svf->g = g;
	svf->k = k;
	svf->a1 = a1;
	svf->a2 = a2;
	svf->a3 = a3;
	
	svf->tick = &tSVFTick;
	svf->setQ = &tSVFSetQ;
	svf->setFreqFromKnob = &tSVFSetFreq; 
	
	return 0;
}



float tSVFEfficientTick(tSVF *svf, float v0) {
	
	float v1,v2,v3;
	float low,high;
	v3 = v0 - svf->ic2eq;
	v1 = (svf->a1 * svf->ic1eq) + (svf->a2 * v3);
	v2 = svf->ic2eq + (svf->a2 * svf->ic1eq) + (svf->a3 * v3);
	svf->ic1eq = (2.0f * v1) - svf->ic1eq;
	svf->ic2eq = (2.0f * v2) - svf->ic2eq;
	
	if (svf->type == SVFTypeLowpass) {
		
		return v2;
	} else if (svf->type == SVFTypeBandpass) {
		
		return v1;
	} else if (svf->type == SVFTypeHighpass) {
		
		return (v0 - (svf->k * v1) - v2);
	} else if (svf->type == SVFTypeNotch) {
		
		return v0 - (svf->k * v1); 
	} else if (svf->type == SVFTypePeak) {
		
		return v0 - (svf->k * v1) - 2.0f * v2;
	} else {
		
		return 0.0f;
	}
	
}

int tSVFEfficientSetFreq(tSVF *svf, uint16_t cutoffKnob) {
	
	svf->g = filtertan[cutoffKnob];
	svf->a1 = 1.0f/(1.0f + svf->g * (svf->g + svf->k));
	svf->a2 = svf->g * svf->a1;
	svf->a3 = svf->g * svf->a2;
	
	return 0;
}

int tSVFEfficientSetQ(tSVF *svf, float Q) {
	
	svf->k = 1.0f/Q;
	svf->a1 = 1.0f/(1.0f + svf->g * (svf->g + svf->k));
	svf->a2 = svf->g * svf->a1;
	svf->a3 = svf->g * svf->a2;
	
	return 0;
}

int tSVFEfficientInit(tSVFEfficient *svf, float sr, SVFType type, uint16_t cutoffKnob, float Q) {
	
	svf->inv_sr = 1.0f/sr;
	svf->type = type;
	
	svf->ic1eq = 0;
	svf->ic2eq = 0;
	
	float a1,a2,a3,g,k;
	g = filtertan[cutoffKnob]; 
	k = 1.0f/Q;
	a1 = 1.0f/(1.0f+g*(g+k));
	a2 = g*a1;
	a3 = g*a2;
	
	svf->g = g;
	svf->k = k;
	svf->a1 = a1;
	svf->a2 = a2;
	svf->a3 = a3;
	
	svf->tick = &tSVFEfficientTick;
	svf->setQ = &tSVFEfficientSetQ;
	svf->setFreqFromKnob = &tSVFEfficientSetFreq; 
	
	return 0;
}

int tSetDelay(tDelay *d, float delay)
{
  float outPointer;

	outPointer = (float)d->in_index - delay;  // read chases write

  while (outPointer < 0) {
    outPointer += DELAY_BUFFER_SIZE; // modulo maximum length
	}

  d->out_index = (uint32_t) outPointer;  // integer part
  d->bottomFrac = outPointer - (float)d->out_index; // fractional part
  d->topFrac = 1.0f - d->bottomFrac;
	
	return 0;
}

float tDelayTick(tDelay *d, float sample)
{
	float output_sample;
  d->buff[(d->in_index)] = sample;

	/* Interpolation */
	output_sample = d->buff[d->out_index] * d->topFrac;
	if (d->out_index+1 < DELAY_BUFFER_SIZE)
		output_sample += d->buff[d->out_index+1] * d->bottomFrac;
	else
		output_sample += d->buff[0] * d->bottomFrac;

	// Increment pointers.
	d->in_index = d->in_index + 1;
	d->out_index = d->out_index  +1;
  if (d->in_index == DELAY_BUFFER_SIZE) d->in_index -= DELAY_BUFFER_SIZE;
  if (d->out_index >= DELAY_BUFFER_SIZE) d->out_index -= DELAY_BUFFER_SIZE;

  return output_sample;
}


/* Delay */
int tDelayInit(tDelay *d, float *buff) {
	d->buff = buff;
	d->bottomFrac = 0.0f;
	d->topFrac = 0.0f;
	d->in_index = 0;
	d->out_index = 0;
	d->tick = &tDelayTick;
	d->setDelay = &tSetDelay;
	return 0;
}

/* Envelope detect */
float tEnvelopeFollowerTick(tEnvelopeFollower *ef, float x) {

	if(x < 0.0f ) x = -x;  /* Absolute value. */

	if ((x >= ef->y) && (x > ef->a_thresh))
	{
		 /* When we hit a peak, ride the peak to the top. */
		ef->y = x;
	}
	else
	{
		 /* Exponential decay of output when signal is low. */
		 //ef->y = envelope_pow[(uint16_t)(ef->y * (float)UINT16_MAX)] * ef->d_coeff; //not quite the right behavior - too much loss of precision?
		 //ef->y = powf(ef->y, 1.000009f) * ef->d_coeff;  // too expensive
		 ef->y = ef->y * ef->d_coeff; // original formula
		 /*
		 ** When output gets close to 0.0, set output to 0.0 to prevent FP underflow
		 ** which can cause a severe performance degradation due to a flood
		 ** of interrupts.
		 */
		 if( ef->y < VERY_SMALL_FLOAT) { 
			 ef->y = 0.0f;

		 }
	}
	
	return ef->y;
	
}

int tDecayCoeff(tEnvelopeFollower *ef, float decayCoeff) {
	return ef->d_coeff = decayCoeff;
}

int tAttackThresh(tEnvelopeFollower *ef, float attackThresh) {
	return ef->a_thresh = attackThresh;
}

int tEnvelopeFollowerInit(tEnvelopeFollower *ef, float attackThreshold, float decayCoeff) {
	ef->y = 0.0f;
	ef->a_thresh = attackThreshold;
	ef->d_coeff = decayCoeff;
	ef->decayCoeff = &tDecayCoeff;
	ef->attackThresh = &tAttackThresh;
	ef->tick = &tEnvelopeFollowerTick;
	return 0;
}

/* Highpass */ 
int tHighpassFreq(tHighpass *hp, float freq) {
	
	hp->R = (1.0f-((freq * 2.0f * 3.14f)*hp->inv_sr));
	
	return 0;
}

// From JOS DC Blocker
float tHighpassTick(tHighpass *hp, float x) {
	
	hp->ys = x - hp->xs + hp->R * hp->ys;
	hp->xs = x;
	return hp->ys;
}

int tHighpassInit(tHighpass *hp, float sr, float freq) {
	
	hp->inv_sr = 1.0f/sr;
	hp->R = (1.0f-((freq * 2.0f * 3.14f)*hp->inv_sr));
	hp->ys = 0.0f;
	hp->xs = 0.0f;
	hp->setFreq = &tHighpassFreq;
	hp->tick = &tHighpassTick;
	
	return 0;
}

/* Cycle */
static int tCycleFreq(tCycle *c, float freq) {	
	
	c->inc = freq * c->inv_sr;
	
	return 0;
}

static float tCycleTick(tCycle *c) {
	
	// Phasor increment
	c->phase += c->inc;
	if (c->phase >= 1.0f) c->phase -= 1.0f;

	// Wavetable synthesis
	float temp = c->wtlen * c->phase;
	int intPart = (int)temp;
	float fracPart = temp - (float)intPart;
	float samp0 = c->wt[intPart];
	if (++intPart >= c->wtlen) intPart = 0;
	float samp1 = c->wt[intPart];
	return (samp0 + (samp1 - samp0) * fracPart);
}

int tCycleInit(tCycle *c, float sr, const float *table, int len) {
	// Underlying phasor
	c->inc = 0.0f;
	c->phase = 0.0f;
	c->inv_sr = 1.0f/sr;
	
	c->wt = table; 
	c->wtlen = len;
	c->setFreq = &tCycleFreq;
	c->tick = &tCycleTick;
	
	return 0; 
}

/* Sawtooth */
static int tSawtoothFreq(tSawtooth *s, float freq) {	
	
	s->inc = freq * s->inv_sr;
	
	return 0;
}

static float tSawtoothTick(tSawtooth *s) {
	// Phasor increment
	s->phase += s->inc;
	if (s->phase >= 1.0f) s->phase -= 1.0f;
	
	return ((s->phase * 2.0f) - 1.0f); 
}


int tSawtoothInit(tSawtooth *s, float sr) {
	// Underlying phasor
	s->inc = 0.0f;
	s->phase = 0.0f;
	s->inv_sr = 1.0f/sr;
	
	s->setFreq = &tSawtoothFreq;
	s->tick = &tSawtoothTick;
	
	return 0; 
}

/* Triangle */
static int tTriangleFreq(tTriangle *t, float freq) {	
	
	t->inc = freq * t->inv_sr;
	
	return 0;
}

static float tTriangleTick(tTriangle *t) {
	// Phasor increment
	t->phase += t->inc;
	if (t->phase >= 1.0f) t->phase -= 1.0f;
	
	float phase =  t->phase * 2.0f; 
	if (phase > 1.0f) phase = 2.0f - phase; 
	phase = (phase * 2.0f) - 1.0f;
	return phase;
	
}

int tTriangleInit(tTriangle *t, float sr) {
// Underlying phasor
	t->inc = 0.0f;
	t->phase = 0.0f;
	t->inv_sr = 1.0f/sr;
	
	t->setFreq = &tTriangleFreq;
	t->tick = &tTriangleTick;
	
	return 0; 
}

/* Square */
static int tPulseWidth(tPulse *pl, float pwidth) {
	//pwidth [0.0, 1.0)
	
	pl->pw = clipAU(0.05f,pwidth,0.95f);
	
	return 0;
}

static int tPulseFreq(tPulse *pl, float freq) {	
	
	pl->inc = freq * pl->inv_sr;
	
	return 0;
}

static float tPulseTick(tPulse *pl) {
	// Phasor increment
	pl->phase += pl->inc;
	if (pl->phase >= 1.0f) pl->phase -= 1.0f;
	
	
	if (pl->phase < pl->pw) return 1.0f;
	else return -1.0f;
}


int tPulseInit(tPulse *pl, float sr, float pwidth) {
	// Underlying phasor
	pl->inc = 0.0f;
	pl->phase = 0.0f;
	pl->inv_sr = 1.0f/sr;
	
	pl->pw = clipAU(0.05f,pwidth,0.95f);
	pl->pwidth = &tPulseWidth;
	pl->setFreq = &tPulseFreq;
	pl->tick = &tPulseTick;
	return 0; 
}


float tPinkNoiseTick(tNoise *n) {
		float tmp;
	  float startWhite = n->rand();
		n->pinkb0 = 0.99765f * n->pinkb0 + startWhite * 0.0990460f;
		n->pinkb1 = 0.96300f * n->pinkb1 + startWhite * 0.2965164f;
		n->pinkb2 = 0.57000f * n->pinkb2 + startWhite * 1.0526913f;
		tmp = n->pinkb0 + n->pinkb1 + n->pinkb2 + startWhite * 0.1848f;
	  return (tmp * 0.05f);
}

float tWhiteNoisetick(tNoise *n) {
	return n->rand();
}

int tNoiseInit(tNoise *n, float sr, float (*randomNumberGenerator)(), NoiseType type) {
	n->rand = randomNumberGenerator;
	if (type == NoiseTypeWhite) {
		n->tick = &tWhiteNoisetick;
	} else if (type == NoiseTypePink)	{
		n->tick = &tPinkNoiseTick;
	} else {
		n->tick = &tWhiteNoisetick;
	}
	return 0;
}



