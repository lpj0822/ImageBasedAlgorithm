#include "utility.h"
#include <string.h>

int myFgets(char* str, int maxCount, FILE *file)
{
	int maxSize = maxCount - 1;
	int count = 0;
	char data;
	if (str == NULL || file == NULL)
		return 0;

	while (my_fread(&data, sizeof(char), 1, file) == 1)
	{
		str[count] = data;
		count++;
		if (count >= maxSize || data == '\n')
		{
			break;
		}
	}
	return count;
}

#if LOG == 1

void logImageS(const char* fileName, const short *img, const int width, const int height)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    int j = 0;
	int offset = 0;
	char data[64];
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
			offset = sprintf(data, "%d ", img[j]);
			my_fwrite(data, sizeof(char), offset, writeFile);
        }
		my_fwrite("\n", sizeof(char), 1, writeFile);
        img += width;
    }
    my_fclose(writeFile);
}

void logImageU(const char* fileName, const unsigned char *img, const int width, const int height)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    int j = 0;
	int offset = 0;
	char data[64];
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
			offset = sprintf(data, "%d ", img[j]);
			my_fwrite(data, sizeof(char), offset, writeFile);
        }
		my_fwrite("\n", sizeof(char), 1, writeFile);
        img += width;
    }
    my_fclose(writeFile);
}

void logImageUS(const char *fileName, const unsigned short *img, const int width, const int height)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    int j = 0;
	int offset = 0;
	char data[64];
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
			offset = sprintf(data, "%d ", img[j]);
			my_fwrite(data, sizeof(char), offset, writeFile);
        }
		my_fwrite("\n", sizeof(char), 1, writeFile);
        img += width;
    }
    my_fclose(writeFile);
}

void logImageI(const char* fileName, const unsigned int *img, const int width, const int height)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    int j = 0;
	int offset = 0;
	char data[64];
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
			offset = sprintf(data, "%d ", img[j]);
			my_fwrite(data, sizeof(char), offset, writeFile);
        }
		my_fwrite("\n", sizeof(char), 1, writeFile);
        img += width;
    }
    my_fclose(writeFile);
}

void logImageF(const char* fileName, const float *img, const int width, const int height)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    int j = 0;
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            fprintf(writeFile, "%.3f ", img[j]);
        }
		fprintf(writeFile, "%c ", '\n');
        img += width;
    }
    my_fclose(writeFile);
}

void logMultiScalesModels(const SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES])
{
    char fileName[32];
    FILE *writeFile;
    int i = 0;
    int j = 0;
	SoftCascadeIntegralChannelsStage stage;
    for(i = 0; i < NUM_SCALES; i++)
    {
        sprintf(fileName, "my_multi_scales_models%d.txt", i);
        writeFile = my_fopen(fileName, "w");
        fprintf(writeFile, "%d\n", i);
        for(j = 0; j < CASCADE_STAGES_NUM; j++)
        {
            stage = multiScalesModels[i].satges[j];
            fprintf(writeFile, "%f|%d %d %d %d %d %f|%d %d %d %d %d %f %f %f|%d %d %d %d %d %f %f %f\n", stage.cascadeThreshold,
                    stage.weakClassifier.level1Node.feature.channelIndex, stage.weakClassifier.level1Node.feature.box.minX, stage.weakClassifier.level1Node.feature.box.minY,
                    stage.weakClassifier.level1Node.feature.box.maxX, stage.weakClassifier.level1Node.feature.box.maxY,  stage.weakClassifier.level1Node.featureThreshold,
                    stage.weakClassifier.level2TrueNode.feature.channelIndex, stage.weakClassifier.level2TrueNode.feature.box.minX, stage.weakClassifier.level2TrueNode.feature.box.minY,
                    stage.weakClassifier.level2TrueNode.feature.box.maxX, stage.weakClassifier.level2TrueNode.feature.box.maxY, stage.weakClassifier.level2TrueNode.featureThreshold,
                    stage.weakClassifier.level2TrueNode.weightTrueLeaf, stage.weakClassifier.level2TrueNode.weightFalseLeaf, stage.weakClassifier.level2FalseNode.feature.channelIndex,
                    stage.weakClassifier.level2FalseNode.feature.box.minX, stage.weakClassifier.level2FalseNode.feature.box.minY, stage.weakClassifier.level2FalseNode.feature.box.maxX,
                    stage.weakClassifier.level2FalseNode.feature.box.maxY, stage.weakClassifier.level2FalseNode.featureThreshold, stage.weakClassifier.level2FalseNode.weightTrueLeaf,
                    stage.weakClassifier.level2FalseNode.weightFalseLeaf);
        }
        my_fclose(writeFile);
    }
}

void logObjects(const char* fileName, const ObjectDetection *objectDetections, const   int objectCount)
{
    FILE *writeFile = my_fopen(fileName, "w");
    int i = 0;
    for(i = 0; i < objectCount; i++)
    {
            fprintf(writeFile, "%d %d %d %d %f\n", objectDetections[i].x, objectDetections[i].y,
                    objectDetections[i].width, objectDetections[i].height, objectDetections[i].score);
    }
    my_fclose(writeFile);
}

void logAllObjects(FILE *writeFile, const char* fileName, const ObjectDetection *objectDetections, const   int objectCount)
{
    int i = 0;
    fprintf(writeFile, "%s|", fileName);
    for(i = 0; i < objectCount; i++)
    {
            fprintf(writeFile, "%d %d %d %d|", objectDetections[i].x, objectDetections[i].y,
                    objectDetections[i].width, objectDetections[i].height);
    }
    fprintf(writeFile, "\n");
}

#endif