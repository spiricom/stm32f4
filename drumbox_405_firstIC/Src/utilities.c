#include "utilities.h"

#define COMPUTE_INC() r->inc = ((r->dest-r->curr)/r->time * r->inv_sr_ms)*((float)r->samples_per_tick)
 

/* Phasor */
static int tMetroRate(tMetro *metro, float rate) {
	
	
	metro->peak = (rate * 0.001f * metro->sr);
	
	return 0;
}

static int tMetroTick(tMetro *metro) {
	
	metro->phase++;
	
	if (metro->phase >= metro->peak) {
		metro->phase = 0;
		return 1;
	} 

	return 0;
}

int tMetroInit(tMetro *metro, float sr, float rate) {

	metro->phase = 0.0f;
	metro->peak = (rate * 0.001f * sr);
	metro->sr = sr;
	metro->setRate = &tMetroRate;
	metro->tick = &tMetroTick;
	
	return 0; 
}



int tRampSetTime(tRamp *r, float time) {
	r->time = time;
	COMPUTE_INC();
	return 0;
}

int tRampSetDest(tRamp *r, float dest) {
	r->dest = dest;
	COMPUTE_INC();
	return 0;
}

float tRampTick(tRamp *r) {
	r->curr += r->inc;
	
	if (((r->curr >= r->dest) && (r->inc > 0.0f)) || ((r->curr <= r->dest) && (r->inc < 0.0f))) {
		r->inc = 0.0f;
	} 
	
	return r->curr;
}

int tRampInit(tRamp *r, float sr, float time, int samples_per_tick) {
	r->inv_sr_ms = 1.0f/(sr*0.001f);
	r->curr = 0.0f;
	r->dest = 0.0f;
	r->time = time;
	r->samples_per_tick = samples_per_tick;
	COMPUTE_INC();
	r->setTime = &tRampSetTime;
	r->setDest = &tRampSetDest;
	r->tick = &tRampTick;
	return 0;
}
