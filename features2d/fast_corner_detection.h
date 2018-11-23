#ifndef FAST_CORNER_DETECTION_H
#define FAST_CORNER_DETECTION_H

#include "baseAlgorithm/image_data_structure.h"

#define MAX_FAST_CORNERS 10000

/*
The references are:
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
*/
void fast9_16(const unsigned char *image, const int imageWidth, const int imageHeight, int threshold,
             int startX, int endX, int startY, int endY, int* cornerNumber, FloatPoint *fastCorner);

#endif // FAST_CORNER_DETECTION_H
