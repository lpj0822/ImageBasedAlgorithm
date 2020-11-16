#include "angle_bin_compute.h"
#include <math.h>
#include <float.h>

float fastSqrt(float x)
{
	float xhalf = 0.5f * x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	x = x * (1.5f - xhalf * x * x);
	//x = x * (1.5f - xhalf * x * x);
	return 1 / x;
}
