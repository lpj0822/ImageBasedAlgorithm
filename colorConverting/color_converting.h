#ifndef COLOR_CONVERTING_H
#define COLOR_CONVERTING_H

#include "baseAlgorithm/image_data_structure.h"

void rgbToGray(const ElementRGB *rgbImage, int width, int height, unsigned char *grayImage);

void rgbToLuv(const ElementRGB *rgbImage, int width, int height, ElementLUV *luvImage);

void rgbToLab(const ElementRGB *rgbImage, int width, int height, ElementLAB *labImage);

void yuv420ToRgb(const unsigned char *yuvImage, int width, int height, ElementRGB *rgbImage);

#endif // COLOR_CONVERTING_H

