#include "image_resize.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#include "helpers/utility.h"

void computeShrinkedImage(const unsigned char *grayImage, int width, int height, int newWidth, int newHeight, unsigned char *dstImage)
{
#if DEBUG == 1
    assert(grayImage != NULL);
    assert(dstImage != NULL);
#endif
    const int shrinkingFactorX = width / newWidth;
    const int shrinkingFactorY = height / newHeight;
    const int factor = shrinkingFactorX * shrinkingFactorY;
    const int maxInputY = height - 1;
    const int maxInputX = width - 1;
    int row = 0;
    int col = 0;
    float sum = 0;
    unsigned char *rowDst = NULL;
    int inputX = 0;
    int inputY = 0;
    int x = 0;
    int y = 0;
    int tempX = 0;
    int tempY = 0;
    for(row = 0; row < newHeight; row++)
    {
        rowDst = dstImage + row * newWidth;
        for(col = 0; col < newWidth; col++)
        {
            sum = 0;
            inputY = row * shrinkingFactorY;
            inputX = col * shrinkingFactorX;
            for(y = 0; y < shrinkingFactorY; y += 1)
            {
                tempY = intMin(inputY + y, maxInputY);
                for(x = 0; x < shrinkingFactorX; x += 1)
                {
                    tempX = intMin(inputX + x, maxInputX);
                    sum += *(grayImage + tempY * width + tempX);
                }
            }
            sum /= factor; // rescale back to [0, 255]
            rowDst[col] = (unsigned char)round(sum);
        }
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

    int *arrX = (int*)calloc(newWidth, sizeof(int));
    int *arrY = (int*)calloc(newHeight, sizeof(int));
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

    free(arrX);
    free(arrY);
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

    int *arrX = (int*)calloc(newWidth, sizeof(int));
    int *arrY = (int*)calloc(newHeight, sizeof(int));
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

    free(arrX);
    free(arrY);
    arrX = NULL;
    arrY = NULL;
}
