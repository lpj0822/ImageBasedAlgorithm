#include "multi_scales_integral_channels_model.h"
#include <math.h>
#include <float.h>

#include "utility.h"
#include "image_processing.h"
#include "integral_channels_define_data.h"
#include "soft_cascade_integral_channels_stage.h"

static void updateWindowSize(const IntegralChannelsFeature feature, Size *windowSize)
{
	windowSize->x = Pedestrian_MAX(feature.box.maxX, windowSize->x);
	windowSize->y = Pedestrian_MAX(feature.box.maxY, windowSize->y);
}

static Size computeCascadeWindowSize(const SoftCascadeIntegralChannelsStage detectorModel[CASCADE_STAGES_NUM])
{
    unsigned int stageIndex = 0;
    Size windowSize = {0, 0};
    for(stageIndex = 0; stageIndex < CASCADE_STAGES_NUM; stageIndex += 1)
    {
        updateWindowSize(detectorModel[stageIndex].weakClassifier.level1Node.feature, &windowSize);
        updateWindowSize(detectorModel[stageIndex].weakClassifier.level2TrueNode.feature, &windowSize);
        updateWindowSize(detectorModel[stageIndex].weakClassifier.level2FalseNode.feature, &windowSize);
    }
    return windowSize;
}

// Helper method that gives the crucial information for the FPDW implementation
// these numbers are obtained via
// doppia/src/test/objects_detection/test_objects_detection + plot_channel_statistics.py
// (this method is not speed critical)
static float getChannelScalingFactor(const int channelIndex, const float relativeScale)
{
    const Bool use_p_dollar_estimates = TRUE;
    const Bool use_v0_estimates = FALSE;
    const Bool use_no_estimate = FALSE;
    float channel_scaling = 1, up_a = 1, down_a = 1, up_b = 2, down_b = 2;
	float a = 1, b = 2;

	const float lambda = 1.099f, dollarA = 0.89f;

    // FIXME how to propagate here which method is used ?
    // should these values be part of the computing structure ?
    if(relativeScale == 1)
    {
        // when no rescaling there is no scaling factor
        return 1.0f;
    }

    if(use_p_dollar_estimates)
    {
        if(channelIndex <= NUMBER_BINS)
        {
            // Gradient histograms and gradient magnitude
			down_a = dollarA;
            down_b = lambda / (float)log(2);

            // upscaling case is roughly a linear growth
            // these are the ideal values
            up_a = 1; 
			up_b = 0;
        }
        else if((channelIndex >= NUMBER_CHANNELS - 3) && (channelIndex <= NUMBER_CHANNELS - 1))
        {
            // LUV channels, quadratic growth
            // these are the ideal values
            down_a = 1; 
			down_b = 2;
            up_a = 1; 
			up_b = 2;
        }
    }
    else if(use_v0_estimates)
    {
        // num_scales ==  12
        // r = a*(k**b); r: feature scaling factor; k: relative scale
        // HOG	for downscaling r = 0.989*(x**-1.022), for upscaling  r = 1.003*(x**1.372)
        // L	for downscaling r = 0.963*(x**-1.771), for upscaling  r = 0.956*(x**1.878)
        // UV	for downscaling r = 0.966*(x**-2.068), for upscaling  r = 0.962*(x**2.095)

        if(channelIndex <= NUMBER_CHANNELS)
        {
            // Gradient histograms and gradient magnitude
            down_a = 1.0f / 0.989f;
            down_b = 1.022f;
            // upscaling case is roughly a linear growth
            up_a = 1.003f;
            up_b = 1.372f;
        }
        else if(channelIndex == NUMBER_CHANNELS - 3)
        { // L channel, quadratic growth
            // for some strange reason test_objects_detection + plot_channel_statistics.py indicate
            // that the L channel behaves differently than UV channels
            down_a = 1.0f / 0.963f;
            down_b = 1.771f;
            up_a = 0.956f;
            up_b = 1.878f;
        }
        else if(channelIndex == NUMBER_CHANNELS - 2 || channelIndex == NUMBER_CHANNELS - 1)
        {
            // UV channels, quadratic growth
            down_a = 1.0f / 0.966f;
            down_b = 2.068f;
            up_a = 0.962f;
            up_b = 2.095f;
        }
    }
    else if(use_no_estimate)
    {
        // we disregard the scaling and keep the same feature value
        up_a = 1;
        up_b = 0;
        down_a = 1;
        down_b = 0;
    }

    if(relativeScale >= 1)
    {
        // upscaling case
        a = up_a;
        b = up_b;
    }
    else
    {
        // size_scaling < 1, downscaling case
        a = down_a;
        b = down_b;
    }

    channel_scaling = a * (float)pow(relativeScale, b);

    return channel_scaling;
}

// we change the size of the rectangle and
// adjust the threshold to take into the account the slight change in area
static void scaleDecisionStump(DecisionStump *decisionStump, const float relativeScale)
{
    const float channelScalingFactor = getChannelScalingFactor(decisionStump->feature.channelIndex, relativeScale);
	Rect box = decisionStump->feature.box;
    const int originalArea = rectangleArea(box);
    float areaApproximationScalingFactor = 1;
    int newArea = 0;
	float expectedNewArea = 0;
    box = scaleRectangle(box, relativeScale);
    newArea = rectangleArea(box);

    decisionStump->feature.box = box;

    if((newArea != 0) && (originalArea != 0))
    {
        expectedNewArea = originalArea * relativeScale * relativeScale;
        areaApproximationScalingFactor = expectedNewArea / newArea;
    }

    decisionStump->featureThreshold /= areaApproximationScalingFactor;
    decisionStump->featureThreshold *= channelScalingFactor;
}

// we change the size of the rectangle and
// adjust the threshold to take into the account the slight change in area
static void scaleDecisionStumpWithWeight(DecisionStumpWithWeights *decisionStump, const float relativeScale)
{
    const float channelScalingFactor = getChannelScalingFactor(decisionStump->feature.channelIndex, relativeScale);
	Rect box = decisionStump->feature.box;
    const int originalArea = rectangleArea(box);
    int newArea = 0;
    float areaApproximationScalingFactor = 1;
	float expectedNewArea = 0;
    box = scaleRectangle(box, relativeScale);
    newArea = rectangleArea(box);

    decisionStump->feature.box = box;

    if((newArea != 0) && (originalArea != 0))
    {
        expectedNewArea = originalArea * relativeScale * relativeScale;
        areaApproximationScalingFactor = expectedNewArea / newArea;
    }

    decisionStump->featureThreshold /= areaApproximationScalingFactor;
    decisionStump->featureThreshold *= channelScalingFactor;
}

static void getRescaledStages(const SoftCascadeIntegralChannelsStage rawModel[CASCADE_STAGES_NUM], const float relativeScale,
                                SoftCascadeIntegralChannelsStage rescaledModel[CASCADE_STAGES_NUM])
{
    unsigned int stageIndex = 0;
	SoftCascadeIntegralChannelsStage stage;
    for(stageIndex = 0; stageIndex < CASCADE_STAGES_NUM; stageIndex += 1)
    {
            stage = rawModel[stageIndex];
            scaleDecisionStump(&stage.weakClassifier.level1Node, relativeScale);
            scaleDecisionStumpWithWeight(&stage.weakClassifier.level2TrueNode, relativeScale);
            scaleDecisionStumpWithWeight(&stage.weakClassifier.level2FalseNode, relativeScale);
            rescaledModel[stageIndex] = stage;
    }
}

void initSearchRangesData(const float minDetectionWindowScale, const float maxDetectionWindowScale, DetectorSearchRange searchRangesData[NUM_SCALES])
{
#if DEBUG == 1
    assert(MIN_DETECTION_WINDOW_SCALE > 0);
#endif

    unsigned int scaleIndex = 0;
    float scaleLogarithmicStep = 0;
	float scale = minDetectionWindowScale;
	DetectorSearchRange rangeData;
	float tempScale = 0.0f;

    if(NUM_SCALES > 1)
    {
		scaleLogarithmicStep = (float)(log(maxDetectionWindowScale) - log(minDetectionWindowScale)) / (NUM_SCALES - 1);
    }

    for(scaleIndex = 0; scaleIndex < NUM_SCALES; scaleIndex += 1)
    {
        rangeData.detectionWindowScale = scale;
        rangeData.minX = 0;
        rangeData.minY = 0;
        rangeData.maxX = 0;
        rangeData.maxY = 0;

        searchRangesData[scaleIndex] = rangeData;
		tempScale = (float)exp(log(scale) + scaleLogarithmicStep);
		scale = Pedestrian_MIN(maxDetectionWindowScale, tempScale);
    }
}

void computeMultiScaledModel(const SoftCascadeIntegralChannelsModel rawMultiScalesModels[RAW_MODEL_NUM],
                                       DetectorSearchRange searchRangesData[NUM_SCALES],
                                       SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES])
{
    unsigned int scaleIndex=0;
    unsigned int modelIndex = 0;
	// search the nearest scale model ---
    unsigned int nearestModelScaleIndex = 0;
    float minAbsLogScale = FLT_MAX;
    float searchRangeLogScale = 0.0f;
	Size newModelWindowSize;
	float relativeScale = 1.0f;
	float logModelScale = 0.0f;
	float absLogScale = 0.0f;
    for(scaleIndex = 0; scaleIndex < NUM_SCALES; scaleIndex += 1)
    {
        multiScalesModels[scaleIndex].scale = searchRangesData[scaleIndex].detectionWindowScale;
		nearestModelScaleIndex = 0;
		minAbsLogScale = FLT_MAX;
		searchRangeLogScale = (float)log(searchRangesData[scaleIndex].detectionWindowScale);
		for (modelIndex = 0; modelIndex < RAW_MODEL_NUM; modelIndex += 1)
		{
			logModelScale = (float)log(rawMultiScalesModels[modelIndex].scale);
			absLogScale = (float)fabs(searchRangeLogScale - logModelScale);

			if (absLogScale < minAbsLogScale)
			{
				minAbsLogScale = absLogScale;
				nearestModelScaleIndex = modelIndex;
			}
		}

#if LOG == 1
		my_printf("%d Selected model scale %.3f for detection window scale %.3f\n", scaleIndex,
                   rawMultiScalesModels[nearestModelScaleIndex].scale, searchRangesData[scaleIndex].detectionWindowScale);
#endif

        // update the search range scale --
        searchRangesData[scaleIndex].detectionWindowScale /= rawMultiScalesModels[nearestModelScaleIndex].scale;

        //get the rescaled detection cascade --
        relativeScale = searchRangesData[scaleIndex].detectionWindowScale;
        getRescaledStages(rawMultiScalesModels[nearestModelScaleIndex].satges, relativeScale, multiScalesModels[scaleIndex].satges);

        multiScalesModels[scaleIndex].shrinkingFactor = rawMultiScalesModels[nearestModelScaleIndex].shrinkingFactor;

        newModelWindowSize.x = (int)round(rawMultiScalesModels[nearestModelScaleIndex].modelSize.x * relativeScale);
        newModelWindowSize.y = (int)round(rawMultiScalesModels[nearestModelScaleIndex].modelSize.y * relativeScale);

        multiScalesModels[scaleIndex].modelSize = newModelWindowSize;
    }
}

void computeSearchRangesData(const int xStride, const int yStride, const SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES], 
							DetectorSearchRange searchRangesData[NUM_SCALES])
{
    unsigned int scaleIndex = 0;
	float strideScaling = 1.0f;
	int xTempStride = 1;
	int yTempStride = 1;
    for(scaleIndex = 0; scaleIndex < NUM_SCALES; scaleIndex += 1)
    {
        // update the scaled search ranges and strides
        strideScaling = multiScalesModels[scaleIndex].scale * INPUT_TO_CHANNEL_SCALE;
		xTempStride = (int)round(xStride * strideScaling);
		yTempStride = (int)round(yStride * strideScaling);
		searchRangesData[scaleIndex].stride.x = Pedestrian_MAX(1, xTempStride);
		searchRangesData[scaleIndex].stride.y = Pedestrian_MAX(1, yTempStride);
		
		//compute_cascade_window_size;
		searchRangesData[scaleIndex].shrunkDetectionWindowSize = computeCascadeWindowSize(multiScalesModels[scaleIndex].satges);

		searchRangesData[scaleIndex].detectionWindowScale = searchRangesData[scaleIndex].detectionWindowScale * INPUT_TO_CHANNEL_SCALE;
     }
}

void updateSearchRangeData(const Size inputSize, DetectorSearchRange searchRangesData[NUM_SCALES])
{
	unsigned int scaleIndex = 0;
	const int shrunkInputWidth = (int)(inputSize.x * INPUT_TO_CHANNEL_SCALE);
	const int shrunkInputHeight = (int)(inputSize.y * INPUT_TO_CHANNEL_SCALE);
	Size shrunkDetectionWindowSize;
	int tempX = 0;
	int tempY = 0;
	for (scaleIndex = 0; scaleIndex < NUM_SCALES; scaleIndex += 1)
	{
		//flooring/ceiling/rounding is important to avoid being "off by one pixel" in the search range
		shrunkDetectionWindowSize = searchRangesData[scaleIndex].shrunkDetectionWindowSize;
		tempX = shrunkInputWidth - shrunkDetectionWindowSize.x;
		tempY = shrunkInputHeight - shrunkDetectionWindowSize.y;
		searchRangesData[scaleIndex].minX = 0;
		searchRangesData[scaleIndex].minY = 0;
		searchRangesData[scaleIndex].maxX = Pedestrian_MAX(0, tempX);
		searchRangesData[scaleIndex].maxY = Pedestrian_MAX(0, tempY);
	}
}
