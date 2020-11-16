#include "image_processing.h"
#include <string.h>
#include <math.h>

#include "utility.h"

int rectangleArea(const Rect rect)
{
    return (rect.maxX - rect.minX) * (rect.maxY - rect.minY);
}

Rect scaleRectangle(const Rect rect, const float relative_scale)
{
	Rect newRect;
	int tempX1 = 0;
	int tempY1 = 0;
	int tempX2 = 0;
	int tempY2 = 0;
    newRect.minX = (int)round(rect.minX * relative_scale);
    newRect.minY = (int)round(rect.minY * relative_scale);
	tempX1 = newRect.minX + 1;
	tempY1 = newRect.minY + 1;
	tempX2 = (int)round(rect.maxX * relative_scale);
	tempY2 = (int)round(rect.maxY * relative_scale);
	newRect.maxX = Pedestrian_MAX(tempX1, tempX2);
	newRect.maxY = Pedestrian_MAX(tempY1, tempY2);

#if DEBUG == 1
    const int area = rectangleArea(newRect);
    assert(area >= 1);
#endif

    return newRect;
}

void computeDerivativeX(const unsigned char *src, int width, int height, short *dst)
{
    int i = 0;
    int j = 0;
    int searchWidth = width - 2;
    int searcHeight = height - 1;
    const unsigned char *src0 = src;
    short dx = 0;
    short *copyDst = NULL;

    memset(dst, 0 , width * height * sizeof(short));
    copyDst = dst;

    for(i = 0; i <= searcHeight; i++)
    {
        for(j = 1; j <= searchWidth; j++)
        {
            dx = (short) (src0[j - 1] * -1 + src0[j + 1] * 1);
            copyDst[j] = dx;
        }
        src0 += width;
        copyDst += width;
    }
}

void computeDerivativeY(const unsigned char *src, int width, int height, short *dst)
{
    int i = 0;
    int j = 0;
    int searchWidth = width - 1;
    int searchHeight = height - 2;
    const unsigned char *src0 = src;
    const unsigned char * src1 = src + width;
    const unsigned char *src2 = src + width * 2;
    short dx = 0;
    short *copyDst = NULL;

    memset(dst, 0 , width * height * sizeof(short));
    copyDst = dst + width;

    for(i = 1; i <= searchHeight; i++)
    {
        for(j = 0; j <= searchWidth; j++)
        {
            dx = (short)(src0[j] * -1 + src2[j] * 1);
            copyDst[j] = dx;
        }
        src0 += width;
        src1 += width;
        src2 += width;
        copyDst += width;
    }
}

void rgbImageResizeOfNeighborInterpolation(const ElementRGB *rgbImage, int width, int height, int newWidth, int newHeight, ElementRGB *dstImage)
{
#if DEBUG == 1
	assert(rgbImage != NULL);
	assert(dstImage != NULL);
#endif
	const ElementRGB * tempSrc = NULL;
	ElementRGB *tempDst = NULL;
	int x = 0;
	int y = 0;
	float factorWidth = (float)width / newWidth;
	float factorHeight = (float)height / newHeight;

	int *arrX = (int*)my_calloc(newWidth, sizeof(int));
	int *arrY = (int*)my_calloc(newHeight, sizeof(int));
	for (x = 0; x < newWidth; x++)
	{
		arrX[x] = (int)(x * factorWidth);
	}
	for (y = 0; y < newHeight; y++)
	{
		arrY[y] = (int)(y * factorHeight);
	}

	for (y = 0; y < newHeight; y++)
	{
		tempSrc = rgbImage + width * arrY[y];
		tempDst = dstImage + newWidth * y;
		for (x = 0; x < newWidth; x++)
		{
			memcpy(tempDst + x, tempSrc + arrX[x], sizeof(ElementRGB));
		}
	}

	my_free(arrX);
	my_free(arrY);
	arrX = NULL;
	arrY = NULL;
}

void grayImageResizeOfNeighborInterpolation(const unsigned char *grayImage, int width, int height, int newWidth, int newHeight, unsigned char *dstImage)
{
#if DEBUG == 1
	assert(grayImage != NULL);
	assert(dstImage != NULL);
#endif
	const unsigned char * tempSrc = NULL;
	unsigned char *tempDst = NULL;
	int x = 0;
	int y = 0;
	float factorWidth = (float)width / newWidth;
	float factorHeight = (float)height / newHeight;

	int *arrX = (int*)my_calloc(newWidth, sizeof(int));
	int *arrY = (int*)my_calloc(newHeight, sizeof(int));
	for (x = 0; x < newWidth; x++)
	{
		arrX[x] = (int)(x * factorWidth);
	}
	for (y = 0; y < newHeight; y++)
	{
		arrY[y] = (int)(y * factorHeight);
	}

	for (y = 0; y < newHeight; y++)
	{
		tempSrc = grayImage + width * arrY[y];
		tempDst = dstImage + newWidth * y;
		for (x = 0; x < newWidth; x++)
		{
			memcpy(tempDst + x, tempSrc + arrX[x], sizeof(unsigned char));
		}
	}

	my_free(arrX);
	my_free(arrY);
	arrX = NULL;
	arrY = NULL;
}

// Helper function that integrates an image
// We assume the inputs are 2d multi_array views
void integrate(const unsigned char *channel, int channelWidth, int channelHeight, unsigned int *integralChannel)
{
#if DEBUG == 1
    assert(channel != NULL);
    assert(integralChannel != NULL);
#endif

    int row = 0;
    int col = 0;
    int integralWidth = channelWidth + 1;
    int integralHeight = channelHeight + 1;

	unsigned int  *integralChannelPreviousRow = NULL;
	unsigned int  *integralChannelRow = NULL;
	const unsigned char  *channelRow = NULL;

    //first row of the integralChannel, is set to zero
    memset(integralChannel, 0, integralWidth * sizeof(unsigned int));

    // we count rows in the integral_channel, they are shifted by one pixel from the image rows
    for(row = 1; row < integralHeight; row += 1)
    {
        integralChannelPreviousRow = integralChannel + integralWidth * (row - 1);
        integralChannelRow = integralChannel + integralWidth * row;
        channelRow = channel + channelWidth * (row - 1);
        integralChannelRow[0] = 0;
        // integralChannelRow.size() == (channelRow.size() + 1) so everything is fine
        for(col = 0; col < channelWidth; col += 1)
        {
            integralChannelRow[col+1] = channelRow[col] +
                    integralChannelRow[col] + integralChannelPreviousRow[col+1]
                    - integralChannelPreviousRow[col];
        }
    }
}
