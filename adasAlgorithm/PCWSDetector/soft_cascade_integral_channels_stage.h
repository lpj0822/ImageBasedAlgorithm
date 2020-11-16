#ifndef SOFT_CASCADE_INTEGRAL_CHANNELS_STAGE_H
#define SOFT_CASCADE_INTEGRAL_CHANNELS_STAGE_H

#include "helpers/utility_data_structure.h"
#include "objects_detection_data_structure.h"

Bool compareDecisionStump(const DecisionStump* stump, const float featureValue);

float compareDecisionStumpWithWeights(const DecisionStumpWithWeights* stump, const float featureValue);

Rect computeWeakClassifierBoundingBox(const Level2DecisionTreeWithWeights weakClassifier);

#endif // SOFT_CASCADE_INTEGRAL_CHANNELS_STAGE_H

