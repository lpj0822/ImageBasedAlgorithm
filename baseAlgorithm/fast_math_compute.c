#include "fast_math_compute.h"
#include <math.h>
#include <float.h>

#include "helpers/utility_data_structure.h"

//use in fastAtan2 function
static const float atan2_p1 = 0.9997878412794807f*(float)(180/MY_PI);
static const float atan2_p3 = -0.3258083974640975f*(float)(180/MY_PI);
static const float atan2_p5 = 0.1555786518463281f*(float)(180/MY_PI);
static const float atan2_p7 = -0.04432655554792128f*(float)(180/MY_PI);

float fastAtan2(float y, float x)
{
    float ax = (float)fabs(x);
    float ay = (float)fabs(y);
    float a, c, c2;
    if(ax >= ay)
    {
        c = ay / (ax + (float)DBL_EPSILON);
        c2 = c * c;
        a = (((atan2_p7 * c2 + atan2_p5) * c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    else
    {
        c = ax / (ay + (float)DBL_EPSILON);
        c2 = c * c;
        a = 90.0f - (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3) * c2 + atan2_p1) * c;
    }
    if( x < 0 )
        a = 180.0f - a;
    if( y < 0 )
        a = - a;
    return a ;
}

float fastSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    //x = x * (1.5f - xhalf * x * x); //improve accuracy
    return 1 / x;
}

float fastInverseSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    //x = x * (1.5f - xhalf * x * x); //improve accuracy
    return x;
}
