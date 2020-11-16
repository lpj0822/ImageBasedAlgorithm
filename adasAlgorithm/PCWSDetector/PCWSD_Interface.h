#ifndef PCWSD_INTERFACE_H
#define PCWSD_INTERFACE_H

typedef struct Image
{
	unsigned char* rgbData;
	int width;
	int height;
}PCWSDImage;

typedef struct PedestrianObject
{
	int x;
	int y;
	int width;
	int height;
	float score;
}PedestrianObject;

typedef struct ObjectSetsPedestrian
{
	PedestrianObject *objects;
	int	nObjectNum;
}objectSetsPedestrian; /* for PCWSD output */

typedef struct PedestrianParam
{
	int imageWidth;
	int imageHeight;
	float minDetectionWindowScale;
	float maxDetectionWindowScale;
	int xStride;
	int yStride;
	float scoreThreshold;
	float minOvelapThreshold;
	int cameraHeight;
	int vanishPointY;
}PCWSDParam;

extern int  PCWSD_Init(const char* modelDirName, const PCWSDParam parameter, objectSetsPedestrian *result);

extern void PCWSD_Processor(const PCWSDImage imageData);

//extern int  PCWSD_Processor_ROI(const PCWSDImage imageData);

extern void PCWSD_GetResult(objectSetsPedestrian *result);

extern void PCWSD_Free(objectSetsPedestrian *result);


#endif