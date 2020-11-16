#ifndef SOFT_CASCADE_INTEGRAL_CHANNELS_MODEL_H
#define SOFT_CASCADE_INTEGRAL_CHANNELS_MODEL_H

#include "helpers/utility_data_structure.h"
#include "objects_detection_data_structure.h"

Bool readModel(const char* fileName, SoftCascadeIntegralChannelsModel *model);

Bool readRawMultiScalesIntegralChannelsModel(const char* dirName, SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM]);

void addCascadeThresholdOffset(SoftCascadeIntegralChannelsModel *model);

void addMultiScalesModelCascadeThresholdOffset(SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM]);

float addLastCascadeThresholdToScoreThreshold(const SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM], float threshold);

// Helper function that searches for the model with scale one and
// sets the detection window size and model_window_to_object_window_converter accordingly
Bool searchScaleOneModel(const SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM], Size *modelSize, Rect *objectWinsow);

#endif // SOFT_CASCADE_INTEGRAL_CHANNELS_MODEL_H

