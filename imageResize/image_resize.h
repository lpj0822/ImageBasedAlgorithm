#ifndef IMAGE_RESIZE_H
#define IMAGE_RESIZE_H

#include "baseAlgorithm/image_data_structure.h"

void computeShrinkedImage(const unsigned char *grayImage, int width, int height, int newWidth, int newHeight, unsigned char *dstImage);

//Nearest neighbor interpolation algorithm
void rgbImageResizeOfNeighborInterpolation(const ElementRGB *rgbImage, int width, int height, int newWidth, int newHeight, ElementRGB *dstImage);
void grayImageResizeOfNeighborInterpolation(const unsigned char *grayImage, int width, int height, int newWidth, int newHeight, unsigned char *dstImage);

#endif // IMAGE_RESIZE_H

