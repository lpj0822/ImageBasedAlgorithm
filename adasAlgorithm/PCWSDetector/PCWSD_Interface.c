#include "PCWSD_Interface.h"
#include <math.h>
#include <string.h>

#include "utility.h"
#include "integral_channels_for_pedestrians.h"
#include "greedy_non_maximal_suppression.h"
#include "soft_cascade_integral_channels_stage.h"
#include "soft_cascade_integral_channels_model.h"
#include "multi_scales_integral_channels_model.h"

#if OPENMP == 1
#include <omp.h>
#endif

static DetectorGlobalParam pedestrianGlobParam;

#if OPENMP == 1
static omp_lock_t lock;
#endif

static int initPedestrainMemory(const Size inputSize, DetectorGlobalParam *globParam)
{
	const Size channelSize = getChannelSize(inputSize, SHRINKING_FACTOR);
	const Size integralChannelSize = { channelSize.x + 1, channelSize.y + 1 };
	const int count = integralChannelSize.x * integralChannelSize.y;
	int index = 0;
	int allMallocSize = 0;
	unsigned char* copyMallPtr = NULL;
	globParam->integralChannelSize = integralChannelSize;
	allMallocSize = NUM_SCALES * sizeof(SoftCascadeIntegralChannelsModel) + \
		NUM_SCALES * sizeof(DetectorSearchRange) + \
		RAW_MAX_DETECTION_COUNT * sizeof(ObjectDetection) + \
		MAX_DETECTION_COUNT * sizeof(ObjectDetection) + \
		NUMBER_CHANNELS * count * sizeof(unsigned int) + \
		count * sizeof(Point);

	allMallocSize += 30 * sizeof(unsigned char);
	globParam->allMemory = (unsigned char *)my_malloc(allMallocSize);
	if (globParam->allMemory == NULL)
		return -1;

	copyMallPtr = globParam->allMemory;
	globParam->multiScalesModels = (SoftCascadeIntegralChannelsModel*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(globParam->multiScalesModels + NUM_SCALES);
	globParam->searchRangesData = (DetectorSearchRange*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(globParam->searchRangesData + NUM_SCALES);
	globParam->rawObjectDetections.outputObjectDetections = (ObjectDetection*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(globParam->searchRangesData + RAW_MAX_DETECTION_COUNT);
	globParam->outputObject.outputObjectDetections = (ObjectDetection*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(globParam->outputObject.outputObjectDetections + MAX_DETECTION_COUNT);

	for (index = 0; index < NUMBER_CHANNELS; index++)
	{
		globParam->integralChannels[index] = (unsigned int *)copyMallPtr;
		copyMallPtr = ALIGN_16BYTE(globParam->integralChannels[index] + count);
	}

	copyMallPtr = ALIGN_16BYTE(globParam->integralChannels[NUMBER_CHANNELS-1] + count);
	globParam->allSlideWindow = (Point *)copyMallPtr;

	return 0;
}

static void initModelWindowToObjectWindowFactor(const Size modelWindowSize, const Rect objectWindow)
{
    const int objectWidth = objectWindow.maxX - objectWindow.minX;
    const int objectHeight = objectWindow.maxY - objectWindow.minY;

#if DEBUG == 1
    const int objectCenter_x = (objectWindow.maxX + objectWindow.minX) / 2;
    const int objectCenter_y = (objectWindow.maxY + objectWindow.minY) / 2;
    // we give a 1 pixel slack to handle impair numbers
    assert(abs(objectCenter_x - (modelWindowSize.x / 2)) <= 1);
    assert(abs(objectCenter_y - (modelWindowSize.y / 2)) <= 1);
#endif

	pedestrianGlobParam.scalingFactorX = (float)objectWidth / modelWindowSize.x;
	pedestrianGlobParam.scalingFactorY = (float)objectHeight / modelWindowSize.y;
}

static void modelWindowToObjectWindow(DetectorOutputObject *detectionObjects)
{
	const int detectionCount = detectionObjects->outputObjectCount;
    int index = 0;
	float halfDeltaX = 0.0f;
	float halfDeltaY = 0.0f;
	float centerX = 0.0f;
	float centerY = 0.0f;
	float halfEpsilonX = 0.0f;
	float halfEpsilonY = 0.0f;
    for(index = 0; index < detectionCount; index++)
    {
        // pedestrian detection
		halfDeltaX = detectionObjects->outputObjectDetections[index].width / 2.0f;
		halfDeltaY = detectionObjects->outputObjectDetections[index].height / 2.0f;
		centerX = detectionObjects->outputObjectDetections[index].x + halfDeltaX;
		centerY = detectionObjects->outputObjectDetections[index].y + halfDeltaY;
        //alpha, beta, gamma, delta, epsilon, zeta, eta...
		halfEpsilonX = halfDeltaX * pedestrianGlobParam.scalingFactorX;
		halfEpsilonY = halfDeltaY * pedestrianGlobParam.scalingFactorY;

		detectionObjects->outputObjectDetections[index].width = (int)(detectionObjects->outputObjectDetections[index].width * pedestrianGlobParam.scalingFactorX);
		detectionObjects->outputObjectDetections[index].height = (int)(detectionObjects->outputObjectDetections[index].height * pedestrianGlobParam.scalingFactorY);
		detectionObjects->outputObjectDetections[index].x = (int)(centerX - halfEpsilonX);
		detectionObjects->outputObjectDetections[index].y = (int)(centerY - halfEpsilonY);  
    }
}

static float getIntegralFeatureValue(const IntegralChannelsFeature feature,
                              unsigned int* const integralChannels[NUMBER_CHANNELS],
                              const Size integralChannelSize,
                              const unsigned int rowIndex,
                              const unsigned int colIndex)
{
    const unsigned int *channel = integralChannels[feature.channelIndex];
    const unsigned int  a = *(channel + integralChannelSize.x * (rowIndex + feature.box.minY) + colIndex + feature.box.minX);
    const unsigned int  b = *(channel + integralChannelSize.x * (rowIndex + feature.box.minY) + colIndex + feature.box.maxX);
    const unsigned int  d = *(channel + integralChannelSize.x * (rowIndex + feature.box.maxY) + colIndex + feature.box.minX);
    const unsigned int  c = *(channel + integralChannelSize.x * (rowIndex + feature.box.maxY) + colIndex + feature.box.maxX);
    const unsigned int featureValue = a + c - b - d;
    return (float)featureValue;
}



static void addDetection(const int detectionCol, const int detectionRow, const float detectionScore,
	const Size detectionWindowSize, DetectorOutputObject *detectionObjects)
{
    int originalCol = 0;
    int originalRow = 0;
    ObjectDetection detection;

    //map the detection point back the input image coordinates
    originalCol = detectionCol * SHRINKING_FACTOR;
    originalRow = detectionRow * SHRINKING_FACTOR;
	//set the detection window
	detection.x = originalCol;
	detection.y = originalRow;
	detection.width = detectionWindowSize.x;
	detection.height = detectionWindowSize.y;
    //set the detection score
    detection.score = detectionScore;
	detectionObjects->outputObjectDetections[detectionObjects->outputObjectCount] = detection;
}

static Bool computeCascadeStage(const SoftCascadeIntegralChannelsStage *stage,
                                unsigned int* const integralChannels[NUMBER_CHANNELS],
                                const Size integralChannelSize,
                                const Point topPoint,
                                float* detectionsScore)
{
    //level 1 nodes return a boolean value,
    //level 2 nodes return directly the float value to add to the score
	float featureValue = getIntegralFeatureValue(stage->weakClassifier.level1Node.feature,
                                         integralChannels, integralChannelSize, topPoint.y, topPoint.x);
	Bool compareResult = compareDecisionStump(&stage->weakClassifier.level1Node, featureValue);
	if (compareResult == TRUE)
    {
		featureValue = getIntegralFeatureValue(stage->weakClassifier.level2TrueNode.feature,
                                               integralChannels, integralChannelSize, topPoint.y, topPoint.x);
		*detectionsScore = *detectionsScore + compareDecisionStumpWithWeights(&stage->weakClassifier.level2TrueNode, featureValue);
    }
    else
    {
		featureValue = getIntegralFeatureValue(stage->weakClassifier.level2FalseNode.feature,
                                               integralChannels, integralChannelSize, topPoint.y, topPoint.x);
		*detectionsScore = *detectionsScore + compareDecisionStumpWithWeights(&stage->weakClassifier.level2FalseNode, featureValue);
    }
    if(*detectionsScore < stage->cascadeThreshold)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

static void detectionPerWindow(const SoftCascadeIntegralChannelsModel *softCascadeModel,
                               unsigned int* const integralChannels[NUMBER_CHANNELS], const Size integralChannelsSize,
							   const Point slideWindow, DetectorOutputObject *detectionObjects)
{
    unsigned int stageIndex = 0;
    float detectionsScore = 0;
    Bool detectionResult = 0;
    for(stageIndex = 0; stageIndex < CASCADE_STAGES_NUM; stageIndex++)
    {
		detectionResult = computeCascadeStage(&softCascadeModel->satges[stageIndex], integralChannels, integralChannelsSize, slideWindow, &detectionsScore);
        if(detectionResult == FALSE)
        {
            break;
        }
    }

	if (stageIndex >= CASCADE_STAGES_NUM && 
		detectionsScore >= pedestrianGlobParam.detectionScoreThreshold)
    {
#if OPENMP == 1
        omp_set_lock(&lock);
		if (detectionObjects->outputObjectCount < RAW_MAX_DETECTION_COUNT)
        {
			addDetection(slideWindow.x, slideWindow.y, detectionsScore, softCascadeModel->modelSize, detectionObjects);
			detectionObjects->outputObjectCount += 1;
        }
        omp_unset_lock(&lock);
#else
		if(detectionObjects->outputObjectCount < RAW_MAX_DETECTION_COUNT)
        {
			addDetection(slideWindow.x, slideWindow.y, detectionsScore, softCascadeModel->modelSize, detectionObjects);
			detectionObjects->outputObjectCount += 1;
        }
#endif
    }
}

#if USE_WINDOW_FILTER == 1
static int getHeightOfWorld(int y, int height)
{
    int resultHeight = 0;
    double cosTheta, sinTheta;
	cosTheta = cos(pedestrianGlobParam.theta);
	sinTheta = sin(pedestrianGlobParam.theta);
	y = y - pedestrianGlobParam.vanishPointY;
	resultHeight = (int)((pedestrianGlobParam.cameraHeight * height) / (y * cosTheta - pedestrianGlobParam.focal * sinTheta));
    return resultHeight;
}
#endif

static int getAllSlideWindow(const DetectorSearchRange *searchRange, const Size windowSize, Point *allSlideWindow)
{
    const Point stride = searchRange->stride;
    int row = 0;
    int column = 0;
    int windowCount = 0;
	Point topPoint;
#if USE_WINDOW_FILTER == 1
    int originalRow = 0;
    int originHeight = 0;
    int resultHeight = 0;
#endif
    for(row = searchRange->minY; row < searchRange->maxY; row += stride.y)
    {
#if USE_WINDOW_FILTER == 1
        originalRow = (int)round(row / INPUT_TO_CHANNEL_SCALE);
		originHeight = (int)(windowSize.y * pedestrianGlobParam.scalingFactorY);
        resultHeight = getHeightOfWorld(originalRow + originHeight, originHeight);
		if(resultHeight >= MIN_PEDESTRIAN_HEIGHT && resultHeight <= MAX_PEDESTRIAN_HEIGHT)
        {
            for(column = searchRange->minX; column < searchRange->maxX; column += stride.x)
            {
                topPoint.x = column;
                topPoint.y = row;
                allSlideWindow[windowCount] = topPoint;
                windowCount++;
            }
        }
#else
        for(column = searchRange->minX; column < searchRange->maxX; column += stride.x)
        {
            topPoint.x = column;
            topPoint.y = row;
            allSlideWindow[windowCount] = topPoint;
            windowCount++;
        }
#endif
    }
    return windowCount;
}

static void computeDetectionsAtScale(const SoftCascadeIntegralChannelsModel *softCascadeModel,
                                     unsigned int* const integralChannels[NUMBER_CHANNELS], const Size integralChannelsSize,
                                     const DetectorSearchRange *searchRange, Point *allSlideWindow,
									 DetectorOutputObject *detectionObjects)
{
    int windowCount = 0;
    int windowIndex = 0;

    windowCount = getAllSlideWindow(searchRange, softCascadeModel->modelSize, allSlideWindow);
#if OPENMP == 1
#pragma omp parallel for
#endif 
    for(windowIndex = 0; windowIndex < windowCount; windowIndex++)
    {
		detectionPerWindow(softCascadeModel, integralChannels, integralChannelsSize, allSlideWindow[windowIndex], detectionObjects);
    }
}

static void processRawDetections(DetectorOutputObject *rawObjectDetections, DetectorOutputObject *objectDetections)
{
	objectDetections->outputObjectCount = 0;
	if (rawObjectDetections->outputObjectCount > 0)
	{
		modelWindowToObjectWindow(rawObjectDetections);
		greedyNonMaximalSuppression(rawObjectDetections, pedestrianGlobParam.minimalOverlapThreshold, objectDetections);
	}
}

int  PCWSD_Init(const char* modelDirName, const PCWSDParam parameter, objectSetsPedestrian *result)
{
	Bool myError = FALSE;
	Size inputSize;
    Size scaleOneModelSize;
	Rect scaleOneObjectWindow;
    SoftCascadeIntegralChannelsModel *rawMultiScalesModels = NULL;
    rawMultiScalesModels = (SoftCascadeIntegralChannelsModel *)my_malloc(RAW_MODEL_NUM * sizeof(SoftCascadeIntegralChannelsModel));
	myError = readRawMultiScalesIntegralChannelsModel(modelDirName, rawMultiScalesModels);
	result->objects = NULL;
	result->nObjectNum = 0;
	inputSize.x = parameter.imageWidth;
	inputSize.y = parameter.imageHeight;
	if (myError == TRUE)
    {
		myError = searchScaleOneModel(rawMultiScalesModels, &scaleOneModelSize, &scaleOneObjectWindow);
		if (myError == TRUE)
        {
			if (initPedestrainMemory(inputSize, &pedestrianGlobParam) == 0)
			{
				pedestrianGlobParam.detectionScoreThreshold = addLastCascadeThresholdToScoreThreshold(rawMultiScalesModels, parameter.scoreThreshold);
				initModelWindowToObjectWindowFactor(scaleOneModelSize, scaleOneObjectWindow);

				initSearchRangesData(parameter.minDetectionWindowScale, parameter.maxDetectionWindowScale, pedestrianGlobParam.searchRangesData);
				computeMultiScaledModel(rawMultiScalesModels, pedestrianGlobParam.searchRangesData, pedestrianGlobParam.multiScalesModels);

				computeSearchRangesData(parameter.xStride, parameter.yStride, pedestrianGlobParam.multiScalesModels, pedestrianGlobParam.searchRangesData);

				updateSearchRangeData(inputSize, pedestrianGlobParam.searchRangesData);

				initComputeIntegralChannel();

				pedestrianGlobParam.minimalOverlapThreshold = parameter.minOvelapThreshold;

#if USE_WINDOW_FILTER == 1
				pedestrianGlobParam.cameraHeight = parameter.cameraHeight; //cm
				pedestrianGlobParam.vanishPointY = parameter.vanishPointY; //pixel
#endif

				result->objects = (PedestrianObject*)my_malloc(MAX_DETECTION_COUNT * sizeof(PedestrianObject));
			}
			else
			{
#if LOG == 1
				my_printf("pedestrian malloc fail!\n");
				return 0;
#endif
			}
        }
		else
		{
#if LOG == 1
			my_printf("can't exit one scale model!\n");
#endif
		}
    }
	else
	{
#if LOG == 1
		my_printf("read model fail!\n");
#endif
	}

    my_free(rawMultiScalesModels);
    rawMultiScalesModels = NULL;
	return (int)myError;
}

static void veryFastIntegralChannelsDetector(const ElementRGB *rgbImage, const Size inputSize, DetectorOutputObject *objectDetections)
{
	//compute the integral channels
    unsigned int index = 0;

#if TEST_TIME == 1
	unsigned int startTime = 0;
#endif

	pedestrianGlobParam.rawObjectDetections.outputObjectCount = 0;

#if LOG == 1
	my_printf("compute integral channel start!\n");
#endif
#if TEST_TIME == 1
	startTime = my_gettime();
#endif
	computeIntegralChannel(rgbImage, inputSize, pedestrianGlobParam.integralChannels);
#if TEST_TIME == 1
	my_printf("ICF cost time:%d ms\n", my_gettime() - startTime);
#endif
#if LOG == 1
	my_printf("compute integral channel finish!\n");
#endif

#if OPENMP == 1
    omp_init_lock(&lock);
#endif
#if LOG == 1
	my_printf("detection start\n");
#endif
#if TEST_TIME == 1
	startTime = my_gettime();
#endif
    //for each model
    for(index = 0; index < NUM_SCALES; index++)
    {
		computeDetectionsAtScale(&pedestrianGlobParam.multiScalesModels[index], pedestrianGlobParam.integralChannels, pedestrianGlobParam.integralChannelSize,
			&pedestrianGlobParam.searchRangesData[index], pedestrianGlobParam.allSlideWindow, &pedestrianGlobParam.rawObjectDetections);
    }
#if TEST_TIME == 1
	my_printf("detect cost time:%d ms\n", my_gettime() - startTime);
#endif
#if LOG == 1
	my_printf("detection stop\n");
#endif
#if OPENMP == 1
    omp_destroy_lock(&lock);
#endif

#if LOG == 1
	my_printf("process window start\n");
#endif
#if TEST_TIME == 1
	startTime = my_gettime();
#endif
    //process detect objects
	processRawDetections(&pedestrianGlobParam.rawObjectDetections, objectDetections);
#if TEST_TIME == 1
	my_printf("process detection cost time:%d ms\n", my_gettime() - startTime);
#endif
#if LOG == 1
	my_printf("number of detections (before non maximal suppression)  on this frame == %d(raw), after filtering ==%d\n", pedestrianGlobParam.rawObjectDetections.outputObjectCount, 
		objectDetections->outputObjectCount);
#endif
}

void PCWSD_Processor(const PCWSDImage imageData)
{
	Size inputSize;
	const ElementRGB* rgbImage = (ElementRGB*)imageData.rgbData;
	pedestrianGlobParam.outputObject.outputObjectCount = 0;
	if (imageData.width <= 0 || imageData.width <= 0)
	{
		return;
	}
	inputSize.x = imageData.width;
	inputSize.y = imageData.height;
	veryFastIntegralChannelsDetector(rgbImage, inputSize, &pedestrianGlobParam.outputObject);
}

//int  PCWSD_Processor_ROI(const PCWSDImage imageData)
//{
//	
//}

void PCWSD_GetResult(objectSetsPedestrian *result)
{
	int loop = 0;
	PedestrianObject pedestrianObject;
	ObjectDetection *outputObjectDetections = pedestrianGlobParam.outputObject.outputObjectDetections;
	result->nObjectNum = pedestrianGlobParam.outputObject.outputObjectCount;
	for (loop = 0; loop < result->nObjectNum; loop++)
	{
		pedestrianObject.score = outputObjectDetections[loop].score;
		pedestrianObject.x = outputObjectDetections[loop].x;
		pedestrianObject.y = outputObjectDetections[loop].y;
		pedestrianObject.width = outputObjectDetections[loop].width;
		pedestrianObject.height = outputObjectDetections[loop].height;
		result->objects[loop] = pedestrianObject;
	}
}

void PCWSD_Free(objectSetsPedestrian *result)
{
	if (pedestrianGlobParam.allMemory != NULL)
	{
		my_free(pedestrianGlobParam.allMemory);
		pedestrianGlobParam.allMemory = NULL;
	}
	if (result->objects != NULL)
	{
		my_free(result->objects);
		result->objects = NULL;
	}
}
