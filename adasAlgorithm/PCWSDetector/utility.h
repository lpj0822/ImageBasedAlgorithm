#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>
#include "utils.h"

#define USE_PYRAMID 0
#define USE_WINDOW_FILTER 0
#define OPENMP 0
#define MY_NEON 0
#define DEBUG 0
#define LOG 1
#define TEST_TIME 1

#if MY_NEON == 1
#include <arm_neon.h>
#endif

int myFgets(char* str, int maxCount, FILE *file);

#define Pedestrian_MIN(num1, num2) ((num1) > (num2) ? (num2):(num1))
#define Pedestrian_MAX(num1, num2) ((num1) > (num2) ? (num1):(num2))

#define ALIGN_16BYTE(x)		((unsigned char*)(((unsigned int)(x) + 15) >> 4 << 4))

#if LOG == 1
#include "objects_detection_data_structure.h"

void logImageS(const char* fileName, const short *img, const int width, const int height);

void logImageUS(const char *fileName, const unsigned short *img, const int width, const int height);

void logImageU(const char* fileName, const unsigned char *img, const int width, const int height);

void logImageI(const char* fileName, const unsigned int *img, const int width, const int height);

void logImageF(const char* fileName, const float *img, const int width, const int height);

void logMultiScalesModels(const SoftCascadeIntegralChannelsModel multiScalesModels[NUM_SCALES]);

void logObjects(const char* fileName, const ObjectDetection *objectDetections, const   int objectCount);

void logAllObjects(FILE *writeFile, const char* fileName, const ObjectDetection *objectDetections, const   int objectCount);

#endif

#if DEBUG == 1

#ifndef assert

#define assert(condition)  \
{\
    if(condition)\
         ;\
        else\
        {\
			my_printf("\n[ERROR]Assert failed: %s,line %u\n", __FILE__, __LINE__);\
            while (1);\
        }\
}

#endif

#endif

#endif // UTILITY_H

