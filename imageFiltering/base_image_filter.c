#include  "base_image_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void gaussianSmoothGray(const unsigned char* src, const int width, const int height,
    const float *kernel, const int kernelSize, unsigned char* dst)
{
    const int center = kernelSize >> 1;
    const int count = width * height;
    int row = 0;
    int col = 0;
    int cc = 0;
    int rr = 0;
    int index = 0;
    float kernelsum = 0;
    float sum = 0;
    unsigned char *tempDst = (unsigned char*)malloc(count * sizeof(unsigned char));

    memset(tempDst, 0, count * sizeof(unsigned char));

    //Blur in the x direction.
    for (row = 0; row < height; row++)
    {
        for (col = 0; col < width; col++)
        {
            kernelsum = 0;
            sum = 0;
            for (cc = (-center); cc <= center; cc++)
            {
                if (((col + cc) >= 0) && ((col + cc) < width))
                {
                    sum += src[row*width + (col + cc)] * kernel[center + cc];
                    kernelsum += kernel[center + cc];
                }
            }
            tempDst[index] = (unsigned char)round(sum / kernelsum);
            index++;
        }
    }

    //Blur in the y direction.
    index = 0;
    for (row = 0; row < height; row++)
    {
        for (col = 0; col < width; col++)
        {
            kernelsum = 0;
            sum = 0;
            for (rr = (-center); rr <= center; rr++)
            {
                if (((row + rr) >= 0) && ((row + rr) < height))
                {
                    sum += tempDst[(row + rr)*width + col] * kernel[center + rr];
                    kernelsum += kernel[center + rr];
                }
            }
            dst[index] = (unsigned char)round(sum / kernelsum);
            index++;
        }
    }
    free(tempDst);
    tempDst = NULL;
}
