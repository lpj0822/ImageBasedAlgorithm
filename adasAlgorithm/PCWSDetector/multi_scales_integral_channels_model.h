#ifndef MULTI_SCALES_INTEGRAL_CHANNELS_MODEL_H
#define MULTI_SCALES_INTEGRAL_CHANNELS_MODEL_H

#include "objects_detection_data_structure.h"

void initSearchRangesData(const float minDetectionWindowScale, const float maxDetectionWindowScale, DetectorSearchRange searchRangesData[NUM_SCALES]);

void computeMultiScaledModel(const SoftCascadeIntegralChannelsModel rawMultiScalesModels[RAW_MODEL_NUM],
                             DetectorSearchRange searchRangesData[NUM_SCALES],
                             SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES]);

void computeSearchRangesData(const int xStride, const int yStride, const SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES], 
							DetectorSearchRange searchRangesData[NUM_SCALES]);

void updateSearchRangeData(const Size inputSize, DetectorSearchRange searchRangesData[NUM_SCALES]);


#endif // MULTI_SCALES_INTEGRAL_CHANNELS_MODEL_H

