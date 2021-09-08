#include "embedded_util.h"

void initMAF(MovingAverageFilter* m, float* dataBuffer, int bufferSize)
{
	m->dataBuffer = dataBuffer;
	m->index = 0;
	m->bufferSize = bufferSize;
}

float pushMAF(MovingAverageFilter* m, float data)
{
	m->dataBuffer[m->index % m->bufferSize] = data;

	m->index++;

	if (m->index < m->bufferSize)
	{
		m->output = data;
	}
	else
	{
		float sum = 0;
		for (int i = 0; i < m->bufferSize; i++)
		{
			sum += m->dataBuffer[i];
		}

		m->output = sum / m->bufferSize;
	}

	return m->output;
}

float getMAFOutput(MovingAverageFilter* m)
{
	return m->output;
}

bool isMAFSufficientlyAccumulated(MovingAverageFilter* m)
{
	return (m->index >= m->bufferSize);
}

float map(float x, float inMin, float inMax, float outMin, float outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void initIIR(IIRFilter* f, int order, float* a, float* b, float* dataBuffer, float* outputBuffer)
{
	f->order = order;
	f->a = a;
	f->b = b;
	f->dataBuffer = dataBuffer;
	f->outputBuffer = outputBuffer;
	f->index = 0;

	for (int i = 0; i < order + 1; i++)
	{
		f->dataBuffer[i] = 0;
		f->outputBuffer[i] = 0;
	}
}
float pushIIR(IIRFilter* f, float data)
{
	if (f->index < (f->order + 1))
	{
		f->output = data;
		f->dataBuffer[f->index] = data;
		f->index++;
	}
	else
	{
		//shift old data and accommodate new
		for (int i = 0; i < f->order; i++)
		{
			f->dataBuffer[i] = f->dataBuffer[i + 1];
			f->outputBuffer[i] = f->outputBuffer[i + 1];
		}
		f->dataBuffer[f->order] = data;

		//get output
		float output = 0;
		for (int i = 0; i <= f->order; i++) output += (f->b[i] * f->dataBuffer[f->order - i]);
		for (int i = 1; i <= f->order; i++) output -= (f->a[i] * f->outputBuffer[f->order - i]);

		f->outputBuffer[f->order] = output;
		f->output = output;
	}
	return f->output;
}
float getIIROutput(IIRFilter* f)
{
	return f->output;
}
bool isIIRSufficientlyAccumulated(IIRFilter* f)
{
	return f->index >= (f->order + 1);
}

void initMedianFilter(MedianFilter* m, float* dataBuffer, int bufferSize)
{
	m->dataBuffer = dataBuffer;
	m->bufferSize = bufferSize;
	m->index = 0;
}
float pushMedianFilter(MedianFilter* m, float data)
{
	m->dataBuffer[m->index % m->bufferSize] = data;

	m->index++;

	if (m->index < m->bufferSize)
	{
		m->output = data;
	}
	else
	{
		//Bubble sort
		for (int i = 0; i < m->bufferSize; i++)
		{
			for (int j = 0; j < m->bufferSize-i-1; j++)
			{
				if (m->dataBuffer[j] > m->dataBuffer[j+1])
				{
					//swap
					float temp = m->dataBuffer[j + 1];
					m->dataBuffer[j + 1] = m->dataBuffer[j];
					m->dataBuffer[j] = temp;
				}
			}
		}

		m->output = m->dataBuffer[(int)(m->bufferSize / 2)];
	}

	return m->output;
}
float getMedianFilterOutput(MedianFilter* m)
{
	return m->output;
}
bool isMedianFilterSufficientlyAccumulated(MedianFilter* m)
{
	return (m->index >= m->bufferSize);
}

void initPID(PID* dst)
{
	dst->errorSum = 0;
	dst->errorSumLimit = 0;
	dst->kP = 0;
	dst->kI = 0;
	dst->kD = 0;
	dst->pastError = 0;
	dst->pastOutput = 0;
	dst->pastValue = 0;
	dst->underOfPoint = 1;
	dst->outputLimit = 0;
}

void updatePID(PID* dst, long target, long input)
{
	dst->nowValue = input;
	dst->target = target;
	dst->nowError = dst->nowValue - dst->target;
	dst->errorSum += dst->nowError;
	dst->errorDiff = dst->nowError - dst->pastError;
	if (dst->errorSumLimit != 0)
	{
		if (dst->errorSum > dst->errorSumLimit)
			dst->errorSum = dst->errorSumLimit;
		else if (dst->errorSum < -dst->errorSumLimit)
			dst->errorSum = -dst->errorSumLimit;
	}
	dst->nowOutput =
		dst->kP * dst->nowError +
		dst->kI * dst->errorSum +
		dst->kD * dst->errorDiff;
	if (dst->underOfPoint == 0) return;
	dst->nowOutput /= dst->underOfPoint;
	dst->pastError = dst->nowError;

	if (dst->outputLimit != 0)
	{
		if (dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
		else if (dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;
	}
}

float limit(float x, float min, float max)
{
	if (x < min) return min;
	if (max < x) return max;
	return x;
}

bool within(float x, float tolerance, float offset)
{
	return (-tolerance < (x - offset) && (x - offset) < tolerance);
}