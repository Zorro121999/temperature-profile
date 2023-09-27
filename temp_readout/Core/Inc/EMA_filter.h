/*
 * EMA_filter.h
 *
 *  Created on: 22.07.2023
 *      Author: user
 */

#ifndef INC_EMA_FILTER_H_
#define INC_EMA_FILTER_H_

typedef struct{
	float alpha;
	float out;
} EMA;

void EMA_Init(EMA *filt, float alpha);

float EMA_Update(EMA *filt, float inp, float alpha);

#endif /* INC_EMA_FILTER_H_ */
