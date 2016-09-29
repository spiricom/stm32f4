// Audio Units

#include "audiounits.h"

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
	p->freq = &phasorFreq;
	p->tick = &phasortick;
	
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
		 ef->y *= ef->d_coeff;
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
	c->freq = &tCycleFreq;
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
	
	s->freq = &tSawtoothFreq;
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
	
	t->freq = &tTriangleFreq;
	t->tick = &tTriangleTick;
	
	return 0; 
}

/* Square */
static int tPulseWidth(tPulse *pl, float pwidth) {
	//pwidth [0.0, 1.0)
	float pw = pwidth;
	if (pw >= 1.0f) pw = 0.99f; 
	if (pw <= 0.0f) pw = 0.01f;
	pl->pw = (pw * 2.0f) - 1.0f; 
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
	
	float phase =  (pl->phase * 2.0f)-1.0f; 
	if (phase < pl->pw) return 1.0f;
	else return -1.0f;
}


int tPulseInit(tPulse *pl, float sr, float pwidth) {
	// Underlying phasor
	pl->inc = 0.0f;
	pl->phase = 0.0f;
	pl->inv_sr = 1.0f/sr;
	
	pl->pw = pwidth; 
	pl->pwidth = &tPulseWidth;
	pl->freq = &tPulseFreq;
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

