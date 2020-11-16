#include "color_converting.h"
#include <math.h>
#include <float.h>
#include <string.h>
#include "utility.h"

#define LUV_LOOKUP_TABLE_SIZE 2048

static float LUVLookupTable[LUV_LOOKUP_TABLE_SIZE];

//converting from YUV420 to RGB24
static long int YUV420CrvTable[256];
static long int YUV420CbuTable[256];
static long int YUV420CguTable[256];
static long int YUV420CgvTable[256];
static long int YUV420Table[256];
static unsigned char YUV420Clp[1024]; //for clip in CCIR601

void rgbToGray(const ElementRGB *rgbImage, int width, int height, unsigned char *grayImage)
{
#if DEBUG == 1
    assert(rgbImage != NULL);
    assert(grayImage != NULL);
#endif

    int i = 0;
    int j = 0;
	int gray = 0;
	ElementRGB rgb;

    memset(grayImage, 0, width * height * sizeof(unsigned char));

    for(i=0; i< height; i++)
    {
        for(j=0; j< width; j++)
        {
            rgb = rgbImage[j];
            gray = 1224 * rgb.r + 2404 * rgb.g + 467 * rgb.b + 2048;
            grayImage[j] = (unsigned char)(gray >> 12);
        }
        rgbImage += width;
        grayImage += width;
    }
}

//Helper table to construct a lookup table
//will only compute the root for values in the range [0, 1]
void initLUVLookupTable(void)
{
    int i = 0;
    float powerY = 1.0f / 3.0f;
	float x = 0;
    for(i = 0; i < LUV_LOOKUP_TABLE_SIZE; i += 1)
    {
        x = (float)i / (LUV_LOOKUP_TABLE_SIZE  - 1);
        LUVLookupTable[i] = (float)pow(x, powerY);
    }
}

//this code is based on the equations from
//http://software.intel.com/sites/products/documentation/hpc/ipp/ippi/ippi_ch6/ch6_color_models.html
//and from
//http://www.f4.fhtw-berlin.de/~barthel/ImageJ/ColorInspector//HTMLHelp/farbraumJava.htm
//and from RGB2Luv_f
//https://code.ros.org/trac/opencv/browser/trunk/opencv/modules/imgproc/src/color.cpp
ElementLUV getPixelRgbToLuv(const ElementRGB rgb)
{
    ElementLUV luv_value;
    const float r = rgb.r / 255.0f;
    const float g = rgb.g / 255.0f;
    const float b = rgb.b / 255.0f;

    const float x = 0.412453f * r + 0.35758f * g + 0.180423f * b;
    const float y = 0.212671f * r + 0.71516f * g + 0.072169f * b;
    const float z = 0.019334f * r + 0.119193f * g + 0.950227f * b;

    const float
			uv_divisor = Pedestrian_MAX((x + 15.f * y + 3.f * z), FLT_EPSILON),
            u = 4.f * x / uv_divisor,
            v = 9.f * y / uv_divisor;

    const unsigned int index = (unsigned int)(y * (LUV_LOOKUP_TABLE_SIZE  - 1));
    const float y_cube_root = LUVLookupTable[index];

    const float
			l_value = Pedestrian_MAX(0.0f, ((116.f * y_cube_root) - 16.f)),
            u_value = 13.f * l_value * (u - 0.197833f),
            v_value = 13.f * l_value * (v - 0.46833f);

    //L in [0, 100], U in [-134, 220], V in [-140, 122]
    const float
            scaled_l = l_value * (255.f / 100.f),
            scaled_u = (u_value + 134.f) * (255.f / (220.f + 134.f )),
            scaled_v = (v_value + 140.f) * (255.f / (122.f + 140.f ));

    luv_value.l = (unsigned char)scaled_l;
    luv_value.u = (unsigned char)scaled_u;
    luv_value.v = (unsigned char)scaled_v;

    return luv_value;
}

//void rgbToLuv(const ElementRGB *rgbImage, int width, int height, ElementLUV *luvImage)
//{
//#if DEBUG == 1
//    assert(rgbImage != NULL);
//    assert(luvImage != NULL);
//#endif
//
//    int i = 0;
//    int j = 0;
//	ElementRGB rgb;
//    initLUVLookupTable();
//    for(i = 0; i< height; i++)
//    {
//        for(j = 0; j<width; j++)
//        {
//            rgb = rgbImage[j];
//            luvImage[j] = getPixelRgbToLuv(rgb);
//        }
//        rgbImage += width;
//        luvImage += width;
//    }
//}

//initialize converting table for YUV420 to RGB
static void initYUV420LookupTable(void)
{
    long int crv,cbu,cgu,cgv;
    int i, ind;
    crv = 104597;
    cbu = 132201;
    cgu = 25675;
    cgv = 53279;
    for (i = 0; i < 256; i++)
    {
       YUV420CrvTable[i] = (i - 128) * crv;
       YUV420CbuTable[i] = (i - 128) * cbu;
       YUV420CguTable[i] = (i - 128) * cgu;
       YUV420CgvTable[i] = (i - 128) * cgv;
       YUV420Table[i] = 76309 * (i - 16);
    }

    for (i = 0; i < 384; i++)
       YUV420Clp[i] =0;

    ind=384;
    for (i = 0; i < 256; i++)
       YUV420Clp[ind++] = i;

    ind=640;
    for (i = 0; i < 384;i++)
       YUV420Clp[ind++] = 255;

}

//convert from YUV420 to RGB24
void yuv420ToRgb(const unsigned char *yuvImage, int width, int height, ElementRGB *rgbImage)
{
    int y1, y2, u, v;
    const unsigned char *py1;
    const unsigned char *py2;
    const unsigned char *uData;
    const unsigned char *vData;
    int i,j, c1, c2, c3, c4;
    int index = 0;
    ElementRGB *d1, *d2;

    py1 = yuvImage;
    py2 = py1 + width;
    uData = yuvImage + width * height;
    vData = uData + (width * height >> 2);
    d1 = rgbImage;
    d2= d1 + width;

    initYUV420LookupTable();

    for (j = 0; j < height; j += 2)
    {

       for (i = 0; i < width; i += 2)
       {
           u = *uData;
           v = *vData;
           c1 = YUV420CrvTable[v];
           c2 = YUV420CguTable[u];
           c3 = YUV420CgvTable[v];
           c4 = YUV420CbuTable[u];
           //up-left
           index = (int)(*py1);
           y1 = YUV420Table[index];
           d1->r = YUV420Clp[384+((y1 + c1)>>16)];
           d1->g = YUV420Clp[384+((y1 - c2 - c3)>>16)];
           d1->b = YUV420Clp[384+((y1 + c4)>>16)];
           d1++;
           py1++;
           //down-left
           index = (int)(*py2);
           y2 = YUV420Table[index];
           d2->r = YUV420Clp[384+((y2 + c1)>>16)];
           d2->g = YUV420Clp[384+((y2 - c2 - c3)>>16)];
           d2->b = YUV420Clp[384+((y2 + c4)>>16)];
           d2++;
           py2++;
           //up-right
           index = (int)(*py1);
           y1 = YUV420Table[index];
           d1->r = YUV420Clp[384+((y1 + c1)>>16)];
           d1->g = YUV420Clp[384+((y1 - c2 - c3)>>16)];
           d1->b = YUV420Clp[384+((y1 + c4)>>16)];
           d1++;
           py1++;
           //down-right
           index = (int)(*py2);
           y2 = YUV420Table[index];
           d2->r = YUV420Clp[384+((y2 + c1)>>16)];
           d2->g = YUV420Clp[384+((y2 - c2 - c3)>>16)];
           d2->b = YUV420Clp[384+((y2 + c4)>>16)];
           d2++;
           py2++;

           uData++;
           vData++;
       }

       d1 += width;
       d2 += width;
       py1 += width;
       py2 += width;
    }
}
