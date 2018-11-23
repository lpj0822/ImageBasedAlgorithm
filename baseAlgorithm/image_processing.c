#include "image_processing.h"
#include <string.h>
#include <math.h>

#include "helpers/utility.h"

void computeDerivativeX(const unsigned char *src, int width, int height, short *dst)
{
#if DEBUG == 1
    assert(src != NULL);
    assert(dst != NULL);
#endif

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
#if DEBUG == 1
    assert(src != NULL);
    assert(dst != NULL);
#endif

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

// Helper function that integrates an image
void computeIntegrate(const unsigned char *image, const int imageWidth, const int imageHeight, unsigned int *integralImage)
{
#if DEBUG == 1
    assert(image != NULL);
    assert(integralImage != NULL);
#endif

    int row = 0;
    int col = 0;
    int sum = 0;
    unsigned int  *integralImagePreviousRow = NULL;
    unsigned int  *integralImageRow = NULL;
    const unsigned char *srcData = image;

    for (col = 0; col < imageWidth; col++)
    {
        sum += srcData[col];
        integralImage[col] = sum;
    }

    integralImagePreviousRow = integralImage;
    integralImageRow = integralImage + imageWidth;
    srcData += imageWidth;

    for (row = 1; row < imageHeight; row += 1)
    {
        sum = 0;
        for (col = 0; col < imageWidth; col += 1)
        {
            sum += srcData[col];
            integralImageRow[col] = integralImagePreviousRow[col] + sum;
        }
        integralImagePreviousRow += imageWidth;
        integralImageRow += imageWidth;
        srcData += imageWidth;
    }
}

// Helper function that integrates an image
void integrate(const unsigned char *image, const int imageWidth, const int imageHeight, unsigned int *integralImage)
{
#if DEBUG == 1
    assert(image != NULL);
    assert(integralImage != NULL);
#endif

    int row = 0;
    int col = 0;
    int integralWidth = imageWidth + 1;
    int integralHeight = imageHeight + 1;

    unsigned int  *integralImagePreviousRow = NULL;
    unsigned int  *integralImageRow = NULL;
	const unsigned char  *channelRow = NULL;

    //first row of the integralImage, is set to zero
    memset(integralImage, 0, integralWidth * sizeof(unsigned int));

    // we count rows in the integral_channel, they are shifted by one pixel from the image rows
    for(row = 1; row < integralHeight; row += 1)
    {
        integralImagePreviousRow = integralImage + integralWidth * (row - 1);
        integralImageRow = integralImage + integralWidth * row;
        channelRow = image + imageWidth * (row - 1);
        integralImageRow[0] = 0;
        // integralImageRow.size() == (channelRow.size() + 1) so everything is fine
        for(col = 0; col < imageWidth; col += 1)
        {
            integralImageRow[col+1] = channelRow[col] +
                    integralImageRow[col] + integralImagePreviousRow[col+1]
                    - integralImagePreviousRow[col];
        }
    }
}
