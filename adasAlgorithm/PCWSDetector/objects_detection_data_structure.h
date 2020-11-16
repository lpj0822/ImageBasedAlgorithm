#ifndef OBJECTS_DETECTION_DATA_STRUCTURE_H
#define OBJECTS_DETECTION_DATA_STRUCTURE_H

#include "image_data_structure.h"
#include "integral_channels_define_data.h"
#include "objects_detection_define_data.h"

typedef struct IntegralChannelsFeature
{
	// box over which compute the integral
	Rect box;
	// integral channel index
	int channelIndex;
}IntegralChannelsFeature;

typedef struct DecisionStump
{
	IntegralChannelsFeature feature;
	// thresholding the corresponding rectangle provides a weak binary classifier
	float featureThreshold;
}DecisionStump;

typedef struct DecisionStumpWithWeights
{
	IntegralChannelsFeature feature;
	// thresholding the corresponding rectangle provides a weak binary classifier
	float featureThreshold;
	float weightTrueLeaf;
	float weightFalseLeaf;
}DecisionStumpWithWeights;

typedef struct Level2DecisionTreeWithWeights
{
	DecisionStumpWithWeights level2TrueNode;
	DecisionStumpWithWeights level2FalseNode;
	DecisionStump level1Node;
}Level2DecisionTreeWithWeights;

typedef struct SoftCascadeIntegralChannelsStage
{
	Level2DecisionTreeWithWeights weakClassifier;
	float cascadeThreshold;
}SoftCascadeIntegralChannelsStage;

typedef struct SoftCascadeIntegralChannelsModel
{
	SoftCascadeIntegralChannelsStage satges[CASCADE_STAGES_NUM];
	Rect objectWindow;
	Size modelSize;
	int shrinkingFactor;
	float scale;
}SoftCascadeIntegralChannelsModel;

typedef struct DetectorSearchRange
{
    Point stride;//< scaled x/y stride
    // detection window scale to consider
    float detectionWindowScale;

	Size shrunkDetectionWindowSize;

    int minX;
    int maxX;
    int minY;
    int maxY;
}DetectorSearchRange;

typedef struct Detection
{
	int x;
	int y;
	int width;
	int height;
	// Detector score, the higher the score the higher the detection confidence
	// Score should be normalized across classes (important for non-maximal suppression)
	float score;
}ObjectDetection;

typedef struct DetectorOutputObject
{
	ObjectDetection *outputObjectDetections;
	int outputObjectCount;
}DetectorOutputObject;

typedef struct DetectorGlobalParam
{
	//all memory
	unsigned char *allMemory;
	//pedestrian models
	SoftCascadeIntegralChannelsModel *multiScalesModels;
	//detector search range
	DetectorSearchRange *searchRangesData;
	//output
	DetectorOutputObject rawObjectDetections;
	DetectorOutputObject outputObject;
	//detect
	unsigned int *integralChannels[NUMBER_CHANNELS];
	Size integralChannelSize;
	Point *allSlideWindow;
	float detectionScoreThreshold;
	//process detection
	float minimalOverlapThreshold;
	float scalingFactorX;
	float scalingFactorY;
	//camera info
	int focal;
	int theta;
	int cameraHeight; //cm
	int vanishPointY; //pixel
}DetectorGlobalParam;

#endif // OBJECTS_DETECTION_DATA_STRUCTURE_H

