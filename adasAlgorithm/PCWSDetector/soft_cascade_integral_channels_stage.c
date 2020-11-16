#include "soft_cascade_integral_channels_stage.h"
#include "utility.h"

Bool compareDecisionStump(const DecisionStump* stump, const float featureValue)
{
    if(featureValue >= stump->featureThreshold)
    {
        return TRUE;
    }
    return FALSE;
}

float compareDecisionStumpWithWeights(const DecisionStumpWithWeights* stump, const float featureValue)
{
    return (featureValue >= stump->featureThreshold) ? stump->weightTrueLeaf : stump->weightFalseLeaf;
}

Rect computeWeakClassifierBoundingBox(const Level2DecisionTreeWithWeights weakClassifier)
{
	Rect boundingBox = weakClassifier.level1Node.feature.box;

	const Rect boxA = weakClassifier.level2TrueNode.feature.box;
	const Rect boxB = weakClassifier.level2FalseNode.feature.box;

	boundingBox.minX = Pedestrian_MIN(boundingBox.minX, boxA.minX);
	boundingBox.minX = Pedestrian_MIN(boundingBox.minX, boxB.minX);

	boundingBox.minY = Pedestrian_MIN(boundingBox.minY, boxA.minY);
	boundingBox.minY = Pedestrian_MIN(boundingBox.minY, boxB.minY);

	boundingBox.maxX = Pedestrian_MAX(boundingBox.maxX, boxA.maxX);
	boundingBox.maxX = Pedestrian_MAX(boundingBox.maxX, boxB.maxX);

	boundingBox.maxY = Pedestrian_MAX(boundingBox.maxY, boxA.maxY);
	boundingBox.maxY = Pedestrian_MAX(boundingBox.maxY, boxB.maxY);

    return boundingBox;
}
