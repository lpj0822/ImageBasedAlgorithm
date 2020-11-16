#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING

#include "objects_detection_data_structure.h"

int rectangleArea(const Rect rect);

Rect scaleRectangle(const Rect rect, const float relative_scale);

void computeDerivativeX(const unsigned char *src, int width, int height, short *dst);

void computeDerivativeY(const unsigned char *src, int width, int height, short *dst);

//Nearest neighbor interpolation algorithm
void rgbImageResizeOfNeighborInterpolation(const ElementRGB *rgbImage, int width, int height, int newWidth, int newHeight, ElementRGB *dstImage);
void grayImageResizeOfNeighborInterpolation(const unsigned char *grayImage, int width, int height, int newWidth, int newHeight, unsigned char *dstImage);

// Helper function that integrates an image
// We assume the inputs are 2d multi_array views
void integrate(const unsigned char *channel, int channelWidth, int channelHeight, unsigned int *integralChannel);

#endif // IMAGE_PROCESSING

