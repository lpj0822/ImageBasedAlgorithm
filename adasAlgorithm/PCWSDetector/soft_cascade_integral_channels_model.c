#include "soft_cascade_integral_channels_model.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "utility.h"

#define MAX_LINE 256
#define SPLIT_MAX_COUNT 8
#define SPLIT_STRING_LENGTH 64
#define FILE_NAME_LENGTH 512

static void initSplitStr(char *splitStr[SPLIT_MAX_COUNT])
{
    unsigned int i = 0;
    for (i = 0; i < SPLIT_MAX_COUNT; i++)
    {
		splitStr[i] = (char*)my_calloc(SPLIT_STRING_LENGTH, sizeof(char));
    }
}

static int strsplit(const char *str, char *parts[], const char *delimiter)
{
	size_t i = 0;
	char *part = NULL;
	char *copy = NULL;
	int strLength = 0;

	if (str == NULL)
		return 0;

	strLength = (int)strlen(str);
	copy = (char *)my_calloc(strLength + 1, sizeof(char));
	if (copy == NULL)
		return -1;
	strcpy(copy, str);

	part = strtok(copy, delimiter);
	strcpy(parts[i], part);
	i++;

	while (part)
	{
		part = strtok(NULL, delimiter);
		if (NULL == part)
			break;
		strcpy(parts[i], part);
		i++;
	}

	if (copy)
	{
		my_free(copy);
		copy = NULL;
	}

	return (int)i;
}

static void freeSplitStr(char *splitStr[SPLIT_MAX_COUNT] )
{
    unsigned int i = 0;
    for (i = 0; i < SPLIT_MAX_COUNT; i++)
    {
            if(splitStr[i])
            {
				my_free(splitStr[i]);
                splitStr[i] = NULL;
            }
    }
}

static void readModelGlobalInfo(char* info, SoftCascadeIntegralChannelsModel *model)
{
    unsigned int splitCount = 0;
    char *splitStr[SPLIT_MAX_COUNT] = {NULL};
    initSplitStr(splitStr);
    splitCount = strsplit(info, splitStr, " ");
    if(splitCount >= 2)
    {
        if(strcmp(splitStr[0], "modelSize") == 0)
        {
            model->modelSize.x = atoi(splitStr[1]);
            model->modelSize.y = atoi(splitStr[2]);
        }
        else if(strcmp(splitStr[0], "objectSize") == 0)
        {
            model->objectWindow.minX = atoi(splitStr[1]);
            model->objectWindow.minY = atoi(splitStr[2]);
            model->objectWindow.maxX = atoi(splitStr[3]);
            model->objectWindow.maxY = atoi(splitStr[4]);
        }
        else if(strcmp(splitStr[0], "scale") == 0)
        {
            model->scale = (float)atof(splitStr[1]);
        }
        else if(strcmp(splitStr[0], "shrinkingFactor") == 0)
        {
            model->shrinkingFactor = atoi(splitStr[1]);
        }
    }
    freeSplitStr(splitStr);
}

static Bool readModelDecisionTreeLevel1(const char *node, DecisionStump *level1Node)
{
    unsigned int splitCount = 0;
    char *splitStr[SPLIT_MAX_COUNT] = { NULL };
    Bool largerThanThreshold = TRUE;
    initSplitStr(splitStr);
    splitCount = strsplit(node, splitStr, " ");
    if (splitCount == 8)
    {
        level1Node->featureThreshold = (float)atof(splitStr[1]);
        largerThanThreshold = (Bool)atoi(splitStr[2]);
        level1Node->feature.channelIndex = atoi(splitStr[3]);
        level1Node->feature.box.minX = atoi(splitStr[4]);
        level1Node->feature.box.minY = atoi(splitStr[5]);
        level1Node->feature.box.maxX = atoi(splitStr[6]);
        level1Node->feature.box.maxY = atoi(splitStr[7]);
    }
    freeSplitStr(splitStr);
    return largerThanThreshold;
}

static void readModelDecisionTreeLevel2(const char *node, const float stageWeight, DecisionStumpWithWeights *level2Node)
{
    unsigned int splitCount = 0;
	Bool largerThanThreshold;
    char *splitStr[SPLIT_MAX_COUNT] = { NULL };
    initSplitStr(splitStr);
    splitCount = strsplit(node, splitStr, " ");
    if (splitCount == 8)
    {
        largerThanThreshold = (Bool)atoi(splitStr[2]);
        level2Node->featureThreshold = (float)atof(splitStr[1]);
        if (largerThanThreshold == TRUE)
        {
            level2Node->weightTrueLeaf = stageWeight;
            level2Node->weightFalseLeaf = -stageWeight;
        }
        else
        {
            level2Node->weightTrueLeaf = -stageWeight;
            level2Node->weightFalseLeaf = stageWeight;
        }
        level2Node->feature.channelIndex = atoi(splitStr[3]);
        level2Node->feature.box.minX = atoi(splitStr[4]);
        level2Node->feature.box.minY = atoi(splitStr[5]);
        level2Node->feature.box.maxX = atoi(splitStr[6]);
        level2Node->feature.box.maxY = atoi(splitStr[7]);
    }
    freeSplitStr(splitStr);
}

static void readModelStageInfo(char *info, char *node1, char *node2, char *node3, const size_t stageIndex, SoftCascadeIntegralChannelsModel *model)
{
    unsigned int splitCount = 0;
    char *splitStr[SPLIT_MAX_COUNT] = {NULL};
	Bool largerThanThreshold = FALSE;
	float stageWeight = 0.0f;
    initSplitStr(splitStr);
    splitCount = strsplit(info, splitStr, " ");
    if(splitCount == 3 && strcmp(splitStr[0], "stage") == 0)
    {
        stageWeight = (float)atof(splitStr[1]);
        model->satges[stageIndex].cascadeThreshold = (float)atof(splitStr[2]);
        largerThanThreshold = readModelDecisionTreeLevel1(node1, &model->satges[stageIndex].weakClassifier.level1Node);
        if(largerThanThreshold)
        {
            readModelDecisionTreeLevel2(node2, stageWeight, &model->satges[stageIndex].weakClassifier.level2TrueNode);
            readModelDecisionTreeLevel2(node3, stageWeight, &model->satges[stageIndex].weakClassifier.level2FalseNode);
        }
        else
        {
            readModelDecisionTreeLevel2(node2, stageWeight, &model->satges[stageIndex].weakClassifier.level2FalseNode);
            readModelDecisionTreeLevel2(node3, stageWeight, &model->satges[stageIndex].weakClassifier.level2TrueNode);
        }
    }
    freeSplitStr(splitStr);
}

Bool readModel(const char* fileName, SoftCascadeIntegralChannelsModel *model)
{
		 FILE *readFile = my_fopen(fileName, "r");
         char strLine[MAX_LINE] = "";
         unsigned int strLength = 0;
         unsigned int stageIndex = 0;
		 unsigned int splitCount = 0;
		 char *splitStr[SPLIT_MAX_COUNT] = { NULL };
         if(readFile == NULL)
         {
			 my_printf("%s open error\n", fileName);
             return FALSE;
         }
         while (1)
         {
			 strLength = myFgets(strLine, MAX_LINE, readFile);   
			 if (strLength <= 0)
             {
				 break;
             }
			 if (strLength >= 1)
			 {
				 splitCount = 0;
				 initSplitStr(splitStr);
				 strLine[strLength - 1] = '\0';
				 splitCount = strsplit(strLine, splitStr, "|");
				 if (splitCount == 1)
				 {
					 readModelGlobalInfo(splitStr[0], model);
				 }
				 else if (splitCount == 4)
				 {
					 readModelStageInfo(splitStr[0], splitStr[1], splitStr[2], splitStr[3], stageIndex, model);
					 stageIndex++;
				 }
				 //my_printf("%s\n", strLine);
				 freeSplitStr(splitStr);
			 }
             memset(strLine, 0, sizeof(strLine));
         }
         my_fclose(readFile);
         return TRUE;
}

Bool readRawMultiScalesIntegralChannelsModel(const char* dirName, SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM])
{
        Bool result = FALSE;
        unsigned int modelIndex = 0;
        char flieName[FILE_NAME_LENGTH] = "";
        for(modelIndex = 0; modelIndex < RAW_MODEL_NUM; modelIndex += 1)
        {
            sprintf(flieName, "%s/my_model%d.txt", dirName, (int)modelIndex);
            result = readModel(flieName, &multiScalesModels[modelIndex]);
            if(result == FALSE)
            {
                break;
            }
        }
        return result;
}

void addCascadeThresholdOffset(SoftCascadeIntegralChannelsModel *model)
{
    unsigned int stageIndex = 0;
	const float b = CASCADE_THRESHOLD_OFFSET_DECAY;
    const float a =  -CASCADE_THRESHOLD_OFFSET / (float)(exp(b * (CASCADE_STAGES_NUM - 1)) - 1);
	float offset[CASCADE_STAGES_NUM];

    if((CASCADE_STAGES_NUM == 0) || ((CASCADE_THRESHOLD_OFFSET == 0) && (CASCADE_THRESHOLD_ADDITIVE_OFFSET == 0)))
    {
        return;
	}

    for(stageIndex = 0; stageIndex < CASCADE_STAGES_NUM; stageIndex += 1)
    {
        offset[stageIndex] = a * (float)(exp(b * stageIndex) - 1) - CASCADE_THRESHOLD_ADDITIVE_OFFSET;
        if((stageIndex == (CASCADE_STAGES_NUM / 2)) || (stageIndex == (CASCADE_STAGES_NUM - 1)) || (stageIndex == 0))
        {
#if LOG == 1
			my_printf("add_cascade_offset: at stage %i the offset value is %.5f\n", stageIndex, offset[stageIndex]);
#endif
        }
    }

    for( stageIndex = 0; stageIndex < CASCADE_STAGES_NUM; stageIndex += 1)
    {
        model->satges[stageIndex].cascadeThreshold += offset[stageIndex];
    }
}

void addMultiScalesModelCascadeThresholdOffset(SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM])
{
    unsigned int modelIndex = 0;
    for(modelIndex = 0; modelIndex < RAW_MODEL_NUM; modelIndex += 1)
    {
            addCascadeThresholdOffset(&multiScalesModels[modelIndex]);
    }
}

float addLastCascadeThresholdToScoreThreshold(const SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM], float threshold)
{
	float scoreThreshold = threshold;
    unsigned int modelIndex = 0;
	float lastCascadeThreshold = 0.0f;
    for(modelIndex = 0; modelIndex < RAW_MODEL_NUM; modelIndex += 1)
    {
        lastCascadeThreshold = multiScalesModels[modelIndex].satges[CASCADE_STAGES_NUM - 1].cascadeThreshold;
        if(lastCascadeThreshold > -1E5f)
        {
            scoreThreshold += lastCascadeThreshold;
            break;
        }
    }
    return scoreThreshold;
}

// Helper function that searches for the model with scale one and
// sets the detection window size and model_window_to_object_window_converter accordingly
Bool searchScaleOneModel(const SoftCascadeIntegralChannelsModel multiScalesModels[RAW_MODEL_NUM], Size *modelSize, Rect *objectWinsow)
{
    Bool foundScaleOne = FALSE;
    unsigned int modelIndex = 0;
    for(modelIndex = 0; modelIndex < RAW_MODEL_NUM; modelIndex += 1)
    {
        if(multiScalesModels[modelIndex].scale == 1.0f)
        {
            *modelSize = multiScalesModels[modelIndex].modelSize;
            *objectWinsow = multiScalesModels[modelIndex].objectWindow;
            foundScaleOne = TRUE;
        }
    }

    return foundScaleOne;
}
