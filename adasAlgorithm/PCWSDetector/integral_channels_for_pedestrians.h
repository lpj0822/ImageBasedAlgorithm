#ifndef INTEGRAL_CHANNELS_FOR_PEDESTRIANS_H
#define INTEGRAL_CHANNELS_FOR_PEDESTRIANS_H

#include "objects_detection_data_structure.h"

Size getChannelSize(const Size inputSize, const int shrinkingFactor);
void initComputeIntegralChannel(void);
void computeIntegralChannel(const ElementRGB *rgbImage, const Size inputSize, unsigned int *integralChannels[NUMBER_CHANNELS]);

#endif // INTEGRAL_CHANNELS_FOR_PEDESTRIANS_H

