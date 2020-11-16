#include "integral_channels_for_pedestrians.h"
#include <math.h>
#include <string.h>

#include "utility.h"
#include "image_processing.h"
#include "color_converting.h"
#include "angle_bin_compute.h"
#include "helpers/utility_data_structure.h"

static float angleBinVectors[NUMBER_BINS][2];

static void initAngleBinCompute(void)
{
	const float angle_quantum = MY_PI / (NUMBER_BINS);
	unsigned int binIndex = 0;
	float theta = 0; // theta is in the center of the angle bin
	for (binIndex = 0; binIndex < NUMBER_BINS; binIndex += 1, theta += angle_quantum)
	{
		angleBinVectors[binIndex][0] = (float)cos(theta);
		angleBinVectors[binIndex][1] = (float)sin(theta);
	}
}

// calling convention is the same of atan2
static int angleBinCompute(const int y, const int x)
{
	// no need to explicitly handle the case y and x == 0,
	// this correspond to zero gradient areas, thus whatever the bin, the histograms will not be modified
	int i = 0;
	int index = 0;
	float maxDotProduct = (float)fabs(x * angleBinVectors[0][0] + y * angleBinVectors[0][1]);
	float dotProduct = 0;

	for (i = 1; i < NUMBER_BINS; i += 1)
	{
		dotProduct = (float)fabs(x * angleBinVectors[i][0] + y * angleBinVectors[i][1]);
		if (dotProduct > maxDotProduct)
		{
			maxDotProduct = dotProduct;
			index = i;
		}
	}

	return index;
}

static void computeShrinkedChannel(const unsigned char *grayImage, const Size inputSize, const Size channelSize, const int shrinkingFactor, unsigned char *dstImage)
{
	const int factor = shrinkingFactor * shrinkingFactor;
	const int maxInputY = inputSize.y - 1;
	const int maxInputX = inputSize.x - 1;
	int row = 0;
	int col = 0;
	int sum = 0;
	unsigned char *rowDst = NULL;
	const unsigned char* rowSrc = NULL;
	int inputX = 0;
	int inputY = 0;
	int x = 0;
	int y = 0;
	int tempX = 0;
	int tempY = 0;
	for (row = 0; row < channelSize.y; row++)
	{
		rowDst = dstImage + row * channelSize.x;
		for (col = 0; col < channelSize.x; col++)
		{
			sum = 0;
			inputY = row * shrinkingFactor;
			inputX = col * shrinkingFactor;
			for (y = 0; y < shrinkingFactor; y += 1)
			{
				tempY = Pedestrian_MIN(inputY + y, maxInputY);
				rowSrc = grayImage + tempY * inputSize.x;
				for (x = 0; x < shrinkingFactor; x += 1)
				{
					tempX = Pedestrian_MIN(inputX + x, maxInputX);
					sum += *(rowSrc + tempX);
				}
			}
			sum /= factor; //rescale back to [0, 255]
			rowDst[col] = (unsigned char)sum;
		}
	}
}

static void computeHOGchannels(const unsigned char *grayImage, Size inputSize, unsigned char *inputChannels[NUMBER_CHANNELS])
{
	//6 gradient orientations channels, 1 gradient magnitude channel
	const int count = inputSize.y * inputSize.x;
	int index = 0;
	short *derivativeDx = NULL;
	short *derivativeDy = NULL;

#if OPENMP == 0 && MY_NEON == 0
	short dx = 0;
	short dy = 0;
	int squareMagnitude = 0;
	int magnitude = 0;
	unsigned char magnitudeU8 = 0;
	int angleIndex = 0;
#endif

#if MY_NEON == 1
	short *copyDerivativeDx = NULL;
	short *copyDerivativeDy = NULL;
	short dx = 0;
	short dy = 0;
	int squareMagnitude = 0;
	int magnitude = 0;
	unsigned char magnitudeU8 = 0;
	int angleIndex = 0;
	float32x4_t squareMagnitude1;
	uint32x4_t squareMagnitude_int;
	int16x4_t dx0;
	int32x4_t dx1;
	int16x4_t dy0;
	int32x4_t dy1;
	int32x4_t squareMagnitude0;
#endif

	derivativeDx = (short*)my_calloc(count, sizeof(short));
	derivativeDy = (short*)my_calloc(count, sizeof(short));

	computeDerivativeX(grayImage, inputSize.x, inputSize.y, derivativeDx);
	computeDerivativeY(grayImage, inputSize.x, inputSize.y, derivativeDy);

#if OPENMP == 0 && MY_NEON == 0
	for (index = 0; index < count; index += 1)
	{
		dx = derivativeDx[index];
		dy = derivativeDy[index];
		squareMagnitude = dx * dx + dy * dy;
		magnitude = (int)sqrtf(squareMagnitude * 0.5f);
#if DEBUG == 1
		assert(magnitude < 256);
#endif
		magnitudeU8 = (unsigned char)magnitude;

		//Using atan2 runs at 58 Hz,
		//while angleBinComputer runs at 62 Hz
		angleIndex = angleBinCompute(dy, dx);
		*(inputChannels[angleIndex] + index) = magnitudeU8;
		*(inputChannels[NUMBER_BINS] + index) = magnitudeU8;
	}
#elif MY_NEON == 1
	copyDerivativeDx = derivativeDx;
	copyDerivativeDy = derivativeDy;
	for (index = 0; index < count - 3; index += 4)
	{
		dx0 = vld1_s16(copyDerivativeDx);
		dx1 = vmovl_s16(dx0);
		dy0 = vld1_s16(copyDerivativeDy);
		dy1 = vmovl_s16(dy0);
		squareMagnitude0 = vmulq_s32(dx1, dx1);
		squareMagnitude0 = vmlaq_s32(squareMagnitude0, dy1, dy1);
		squareMagnitude0 = vshrq_n_s32(squareMagnitude0, 1);

		squareMagnitude1 = vcvtq_f32_s32(squareMagnitude0);
		squareMagnitude1 = vrsqrteq_f32(squareMagnitude1);
		squareMagnitude1 = vrecpeq_f32(squareMagnitude1);
		squareMagnitude_int = vcvtq_u32_f32(squareMagnitude1);

		vst1q_u32(inputChannels[NUMBER_BINS] + index, squareMagnitude_int);

		angleIndex = angleBinCompute(copyDerivativeDy[0], copyDerivativeDx[0]);
		*(inputChannels[angleIndex] + index) = (unsigned char)vgetq_lane_u32(squareMagnitude_int, 0);

		angleIndex = angleBinCompute(copyDerivativeDy[1], copyDerivativeDx[1]);
		*(inputChannels[angleIndex] + index + 1) = (unsigned char)vgetq_lane_u32(squareMagnitude_int, 1);

		angleIndex = angleBinCompute(copyDerivativeDy[2], copyDerivativeDx[2]);
		*(inputChannels[angleIndex] + index + 2) = (unsigned char)vgetq_lane_u32(squareMagnitude_int, 2);

		angleIndex = angleBinCompute(copyDerivativeDy[3], copyDerivativeDx[3]);
		*(inputChannels[angleIndex] + index + 3) = (unsigned char)vgetq_lane_u32(squareMagnitude_int, 3);

		copyDerivativeDx += 4;
		copyDerivativeDy += 4;
	}

	for (; index < count; index += 1)
	{
		dx = *copyDerivativeDx;
		dy = *copyDerivativeDy;
		squareMagnitude = dx * dx + dy * dy;
		//magnitude = (int)fastSqrt(squareMagnitude * 0.5f);
		magnitude = (int)sqrtf(squareMagnitude >> 1);
		magnitudeU8 = (unsigned char)magnitude;
		//Using atan2 runs at 58 Hz,
		//while angleBinComputer runs at 62 Hz
		angleIndex = angleBinCompute(dy, dx);
		*(inputChannels[angleIndex] + index) = magnitudeU8;
		*(inputChannels[NUMBER_BINS] + index) = magnitudeU8;
		copyDerivativeDx++;
		copyDerivativeDy++;
	}
#else
#pragma omp parallel for
	for (index = 0; index < count; index += 1)
	{
		const short dx = derivativeDx[index];
		const short dy = derivativeDy[index];
		const int squareMagnitude = dx * dx + dy * dy;
		const int magnitude = (int)sqrtf(squareMagnitude * 0.5f);
#if DEBUG == 1
		assert(magnitude < 256);
#endif
		const unsigned char magnitudeU8 = (unsigned char)magnitude;

		//Using atan2 runs at 58 Hz,
		//while angleBinComputer runs at 62 Hz
		const int angleIndex = angleBinCompute(dy, dx);
		*(inputChannels[angleIndex] + index) = magnitudeU8;
		*(inputChannels[NUMBER_BINS] + index) = magnitudeU8;
	}
#endif

	my_free(derivativeDx);
	my_free(derivativeDy);
	derivativeDx = NULL;
	derivativeDy = NULL;
}

static void computeLUVchannels(const ElementRGB *rgbImage, Size inputSize, unsigned char *inputChannels[NUMBER_CHANNELS])
{
	const int count = inputSize.y * inputSize.x;
	int index = 0;
	unsigned char *inputChannelL = inputChannels[7];
	unsigned char *inputChannelU = inputChannels[8];
	unsigned char *inputChannelV = inputChannels[9];
#if OPENMP == 0
	ElementLUV luv;
	for (index = 0; index < count; index += 1)
	{
		luv = getPixelRgbToLuv(rgbImage[index]);
		inputChannelL[index] = luv.l;
		inputChannelU[index] = luv.u;
		inputChannelV[index] = luv.v;
	}
#else
#pragma omp parallel for
	for (index = 0; index < count; index += 1)
	{
		const ElementLUV luv = getPixelRgbToLuv(rgbImage[index]);
		inputChannelL[index] = luv.l;
		inputChannelU[index] = luv.u;
		inputChannelV[index] = luv.v;
	}
#endif
}

static void resizeChannels(unsigned char * const inputChannels[NUMBER_CHANNELS], const Size inputSize,
	const Size channelSize, unsigned char *channels[NUMBER_CHANNELS])
{
	int channelIndex = 0;
#if OPENMP == 1
#pragma omp parallel for
#endif
	for (channelIndex = 0; channelIndex < NUMBER_CHANNELS; channelIndex += 1)
	{
		computeShrinkedChannel(inputChannels[channelIndex], inputSize, channelSize, SHRINKING_FACTOR, channels[channelIndex]);
	}
}

static void integrateChannels(unsigned char* const channels[NUMBER_CHANNELS], const Size channelSize, unsigned int *integralChannels[NUMBER_CHANNELS])
{
	int channelIndex = 0;
	for (channelIndex = 0; channelIndex < NUMBER_CHANNELS; channelIndex += 1)
	{
		//for some strange reason explicitly defining this reference is needed to get the code compiling
		integrate(channels[channelIndex], channelSize.x, channelSize.y, integralChannels[channelIndex]);
	}
}

Size getChannelSize(const Size inputSize, const int shrinkingFactor)
{
	Size channelSize;
	//shrinkingFactor/2 to round-up
	if (shrinkingFactor == 4)
	{
		channelSize.x = (((inputSize.x + 1) / 2) + 1) / 2;
		channelSize.y = (((inputSize.y + 1) / 2) + 1) / 2;
	}
	else if (shrinkingFactor == 2)
	{
		channelSize.x = (inputSize.x + 1) / 2;
		channelSize.y = (inputSize.y + 1) / 2;
	}
	else
	{
		channelSize = inputSize;
	}

	return channelSize;
}

void initComputeIntegralChannel(void)
{
	//compute hog channels
	initAngleBinCompute();
	//compute LUV channels
	initLUVLookupTable();
}


void computeIntegralChannel(const ElementRGB *rgbImage, const Size inputSize, unsigned int *integralChannels[NUMBER_CHANNELS])
{
#if DEBUG == 1
	assert(rgbImage != NULL);
	assert(integralChannels != NULL);
#endif
	const Size channelSize = getChannelSize(inputSize, SHRINKING_FACTOR);
	unsigned char* grayImage = NULL;
	unsigned char *channels[NUMBER_CHANNELS];
	unsigned char *inputChannels[NUMBER_CHANNELS];
	unsigned int index = 0;

#if TEST_TIME == 1
	unsigned int startTime = 0;
#endif

	for (index = 0; index < NUMBER_CHANNELS; index++)
	{
		//allocate the channel images
		//since  hogInputChannels[angleIndex][y][x] does not set the value for all hog channels,
		//we need to set them all to zero
		inputChannels[index] = (unsigned char*)my_calloc(inputSize.y * inputSize.x, sizeof(unsigned char));
		channels[index] = (unsigned char *)my_calloc(channelSize.y * channelSize.x, sizeof(unsigned char));
		memset(inputChannels[index], 0, inputSize.y * inputSize.x * sizeof(unsigned char));
		memset(channels[index], 0, channelSize.y * channelSize.x * sizeof(unsigned char));
	}
#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	grayImage = (unsigned char*)my_calloc(inputSize.y * inputSize.x, sizeof(unsigned char));
	rgbToGray(rgbImage, inputSize.x, inputSize.y, grayImage);
#if TEST_TIME == 1
	my_printf("gray cost time:%d ms\n", my_gettime() - startTime);
#endif

#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	computeHOGchannels(grayImage, inputSize, inputChannels);
#if TEST_TIME == 1
	my_printf("HOG cost time:%d ms\n", my_gettime() - startTime);
#endif

#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	computeLUVchannels(rgbImage, inputSize, inputChannels);
#if TEST_TIME == 1
	my_printf("LUV cost time:%d ms\n", my_gettime() - startTime);
#endif

#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	resizeChannels(inputChannels, inputSize, channelSize, channels);
#if TEST_TIME == 1
	my_printf("resize cost time:%d ms\n", my_gettime() - startTime);
#endif

#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	integrateChannels(channels, channelSize, integralChannels);
#if TEST_TIME == 1
	my_printf("integrate cost time:%d ms\n", my_gettime() - startTime);
#endif

	my_free(grayImage);
	grayImage = NULL;
	for (index = 0; index < NUMBER_CHANNELS; index++)
	{
		my_free(inputChannels[index]);
		my_free(channels[index]);
		inputChannels[index] = NULL;
		channels[index] = NULL;
	}
}
