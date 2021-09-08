/*
*	Embedded_util written by Jin Kim 2021.
* 
*	This library contains useful math utility and filter implementations for embedded developments.
*	Copyright 2021. Jin Kim. All rights reserved.
* 
*	last updated : Jan 6 2021
*/

#ifndef _EMBEDDED_UTIL_H_
#define _EMBEDDED_UTIL_H_

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

#define PI 3.1415926535897932384626433832795
#define SET(p,n) ((p) |= (1 << (n)))
#define CLR(p,n) ((p) &= (~(1) << (n)))
#define GET(x,n) (x>>n)&1
#define RAD2DEG(x) (x/PI*180.0)
#define DEG2RAD(x) (x/180.0*PI)

typedef unsigned char byte;

typedef struct _MovingAverageFilter
{
	unsigned int index;
	unsigned int bufferSize;
	float* dataBuffer;
	float output;
}MovingAverageFilter;
void	initMAF(MovingAverageFilter* m,	float* dataBuffer, int bufferSize);
float	pushMAF(MovingAverageFilter* m, float data);
float	getMAFOutput(MovingAverageFilter* m);
bool	isMAFSufficientlyAccumulated(MovingAverageFilter* m);

typedef struct _IIRFilter
{
	unsigned int order;
	float* dataBuffer;
	float* outputBuffer;
	float* a;
	float* b;
	unsigned int index;
	float output;
}IIRFilter;
void	initIIR(IIRFilter* f, int order, float* a, float* b, float* dataBuffer, float* outputBuffer);	//size of a, b, dataBuffer, outputBuffer must be (order+1)
float	pushIIR(IIRFilter* f, float data);
float	getIIROutput(IIRFilter* f);
bool	isIIRSufficientlyAccumulated(IIRFilter* f);

typedef struct _MedianFilter
{
	unsigned int index;
	unsigned int bufferSize;
	float* dataBuffer;
	float output;
}MedianFilter;
void initMedianFilter(MedianFilter *m, float* dataBuffer, int bufferSize);
float pushMedianFilter(MedianFilter* m, float data);
float getMedianFilterOutput(MedianFilter* m);
bool isMedianFilterSufficientlyAccumulated(MedianFilter* m);

typedef struct _PID 
{
	long nowValue;
	long pastValue;

	long nowError;
	long pastError;
	long target;

	long errorSum;
	long errorSumLimit;
	long errorDiff;

	long nowOutput;
	long pastOutput;
	long outputLimit;

	long underOfPoint;

	long kP, kI, kD;
}PID;
void updatePID(PID* dst,
	long target,
	long input
);
void initPID(PID* dst
);

float	map(float x, float inMin, float inMax, float outMin, float outMax);
float	limit(float x, float min, float max);
bool	within(float x, float tolerance, float offset);

#ifdef __cplusplus
}
#endif

#endif
