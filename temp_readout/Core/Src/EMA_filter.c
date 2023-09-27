#include "EMA_filter.h"


void EMA_init(EMA *filt, float alpha)
{
	filt->alpha=0.5f;
	filt->out=0.0f;
}

float EMA_Update( EMA *filt,float inp,float alpha)
{
	filt->out=alpha*inp+(1.0f-alpha)*filt->out;
	return filt->out;
}
