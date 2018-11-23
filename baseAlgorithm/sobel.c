#include "sobel.h"
#include <string.h>
#include <stdlib.h>

//use in prefilterYSobel function
#define OFS (1024) // 256*4
#define SOBEL_TABLE (2304) // OFS * 2 + 256
#define FTZERO (80)

static unsigned char sobleTable[SOBEL_TABLE] = {0};

void sobelX(const unsigned char *src, int width, int height, short *dst)
{
#if NEON == 0
    int i = 0;
    int j = 0;
    int searchWidth = width - 2;
    int searchheight = height - 2;
    const unsigned char *src0 = src, *src1 = src + width * 2 , *src_ = src + width ;
    short dx;

    for(j = 0; j<height; j++)
    {
        memset(dst + j * width, 0 , width * sizeof(short));
    }

    short *copyDst = dst + width;

    for(i = 1; i <= searchheight; i++)
    {
        for(j = 1; j <= searchWidth; j++)
        {
            dx = (short)(src0[j + 1] + src_[j + 1] * 2 + src1[j + 1]) - (src0[j - 1] + src_[j - 1] * 2 + src1[j - 1]);
            copyDst[j] = dx;
        }
        src0 += width;
        src1 += width;
        src_ += width;
        copyDst += width;
    }
#elif NEON == 1
    int i = 0;
            int j = 0;
            int searchWidth = width - 2;
            int searchheight = height - 2;
            const unsigned char *src0 = src;
            const unsigned char *src1 = src + width * 2;
            const unsigned char *src_ = src + width ;

            for(j = 0; j<height; j++)
            {
                memset(dst + j * width, 0 , width * sizeof(short));
            }

            short *copyDst = dst + width;
            uint8x8_t dataX2 = vdup_n_u8(2);

            for(i = 1; i <= searchheight; i++)
            {
                for(j = 1; j <= searchWidth; j += 8)
                {
                    uint8x8_t neon_src02 = vld1_u8(src0 + j + 1);
                    uint8x8_t neon_src12 = vld1_u8(src_ + j + 1);
                    uint8x8_t neon_src22 = vld1_u8(src1 + j + 1);

                    uint8x8_t neon_src00 = vld1_u8(src0 + j - 1);
                    uint8x8_t neon_src10 = vld1_u8(src_ + j - 1);
                    uint8x8_t neon_src20 = vld1_u8(src1 + j - 1);

                    uint16x8_t neon_src12_x2 = vmull_u8(neon_src12, dataX2);
                    uint16x8_t neon_src10_x2 = vmull_u8(neon_src10, dataX2);

                    uint16x8_t neon_pre = vaddw_u8(neon_src12_x2, neon_src02);
                    neon_pre = vaddw_u8(neon_pre, neon_src22);

                    uint16x8_t neon_next = vaddw_u8(neon_src10_x2, neon_src00);
                    neon_next = vaddw_u8(neon_next, neon_src20);

                    int16x8_t neon_1 = vreinterpretq_s16_u16(neon_pre);
                    int16x8_t neon_2 = vreinterpretq_s16_u16(neon_next);
                    int16x8_t dx = vsubq_s16(neon_1, neon_2);
                    vst1q_s16(copyDst + j, dx);
                }
                src0 += width;
                src1 += width;
                src_ += width;
                copyDst += width;
            }
#endif
}

void sobelY(const unsigned char *src, int width, int height, short *dst)
{
    int i = 0;
    int j = 0;
    int searchWidth = width - 2;
    int searchheight = height - 2;
    const unsigned char *src0 = src, *src1 = src + width * 2 , *src_ = src + width ;
    short dy;

    for(j = 0; j < height; j++)
    {
        memset(dst + j * width, 0 , width *sizeof(short));
    }

    short *copyDst = dst + width;

    for(i = 1; i <= searchheight; i++)
    {
        for(j = 1; j <= searchWidth; j++)
        {
            dy = (short) (src1[j - 1] + src1[j] * 2 + src1[j + 1])-(src0[j - 1] + src0[j] * 2 + src0[j + 1]);
            copyDst[j] = dy;
        }

        src0 += width;
        src1 += width;
        src_ += width;
        copyDst += width;
    }
}

static void initSobelTable(const int ftzero, unsigned char *sobleTable)
{
    int x = 0;
    for (x = 0; x < SOBEL_TABLE; x++)
        sobleTable[x] = (unsigned char)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero * 2 : x - OFS + ftzero);
}

void prefilterXSobel(const unsigned char* src, const int width, const int height, unsigned char* dxImage)
{
    int col = 0;
    int row = 0;
    int startX = 1;
    int startY = 1;
    int stopY = height - 2;
    int stopX = width - 1;
    int rowWidth = width * 2;
    const unsigned char *srcData0 = src;
    const unsigned char *srcData1 = srcData0 + width;
    const unsigned char *srcData2 = srcData1 + width;
    const unsigned char *srcData3 = srcData2 + width;
    unsigned char *dstData0 = dxImage + width;
    unsigned char *dstData1 = dstData0 + width;
    int d0, d1, d2, d3, v0, v1;
    int index1, index2;

    initSobelTable(FTZERO, sobleTable);

    for( row = startY; row < stopY; row += 2 )
    {
        for(col = startX; col < stopX; col++)
        {
            index1 = col + 1;
            index2 = col - 1;
            d0 = srcData0[index1] - srcData0[index2];
            d1 = srcData1[index1] - srcData1[index2];
            d2 = srcData2[index1] - srcData2[index2];
            d3 = srcData3[index1] - srcData3[index2];
            v0 = sobleTable[d0 + (d1 << 1) + d2 + OFS];
            v1 = sobleTable[d1 + (d2 << 1) + d3 + OFS];

//            dstData0[col] = (unsigned char)v0;
//            dstData1[col] = (unsigned char)v1;

            if (v0 < 50)
            {
                dstData0[col] = 255;
            }
            else
            {
                dstData0[col] = 0;
            }

            if (v1 < 50)
            {
                dstData1[col] = 255;
            }
            else
            {
                dstData1[col] = 0;
            }
        }
        srcData0 += rowWidth;
        srcData1 += rowWidth;
        srcData2 += rowWidth;
        srcData3 += rowWidth;
        dstData0 += rowWidth;
        dstData1 += rowWidth;
    }
}

void  prefilterYSobel(const unsigned char *src, const int width, const int height, unsigned char *dyImage)
{
    int col = 0;
    int row = 0;
    int startX = 1;
    int startY = 1;
    int stopY = height - 1;
    int stopX = width - 2;
    const unsigned char *srcData0 = src;
    const unsigned char *srcData1 = srcData0 + width;
    const unsigned char *srcData2 = srcData1 + width;
    unsigned char *dstData = dyImage + width;
    int d0, d1, d2, d3, v0, v1;
    int index1, index2, index3;

    initSobelTable(FTZERO, sobleTable);

    memset(dyImage, 0 , width * height *sizeof(unsigned char));

    for (row = startY; row < stopY; row++)
    {
        for (col = startX; col < stopX; col += 2)
        {
            index1 = col - 1;
            index2 = col + 1;
            index3 = col + 2;
            d0 = srcData0[index1] - srcData2[index1];
            d1 = srcData0[col] - srcData2[col];
            d2 = srcData0[index2] - srcData2[index2];
            d3 = srcData0[index3] - srcData2[index3];

            v0 = sobleTable[d0 + (d1 << 1) + d2 + OFS];
            v1 = sobleTable[d1 + (d2 << 1) + d3 + OFS];

//            dstData[col] = (unsigned char)v0;
//            dstData[index2] = (unsigned char)v1;

            if (v0 < 50)
            {
                dstData[col] = 255;
            }
            else
            {
                dstData[col] = 0;
            }

            if (v1 < 50)
            {
                dstData[index2] = 255;
            }
            else
            {
                dstData[index2] = 0;
            }
        }
        srcData0 += width;
        srcData1 += width;
        srcData2 += width;
        dstData += width;
    }
}
