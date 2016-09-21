#include "utilities.h"

#define COMPUTE_INC() r->inc = ((r->dest-r->curr)/r->time * r->inv_sr_ms)*((float)r->samples_per_step)

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

float tRampStep(tRamp *r) {
	r->curr += r->inc;
	
	if (((r->curr >= r->dest) && (r->inc > 0.0f)) || ((r->curr <= r->dest) && (r->inc < 0.0f))) {
		r->inc = 0.0f;
	} 
	
	return r->curr;
}

int tRampInit(tRamp *r, float sr, float time, int samples_per_step) {
	r->inv_sr_ms = 1.0f/(sr*0.001f);
	r->curr = 0.0f;
	r->dest = 0.0f;
	r->time = time;
	r->samples_per_step = samples_per_step;
	COMPUTE_INC();
	r->setTime = &tRampSetTime;
	r->setDest = &tRampSetDest;
	r->step = &tRampStep;
	return 0;
}