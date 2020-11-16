#include "greedy_non_maximal_suppression.h"
#include <string.h>

#include "utility.h"
#include "image_processing.h"

static void detectionSwap(ObjectDetection *num1, ObjectDetection *num2)
{
    ObjectDetection tmp = *num1;
    *num1 = *num2;
    *num2 = tmp;
}

static int detectionsPartition(ObjectDetection* array, const int length)
{
    int i = 0;
    int pivot = 0;
    // swap random slot selection to end.
    //array[length-1] will hold the pivot value.
    detectionSwap(array, array + length - 1);
    for (i = 0; i < length; i++)
    {
        if (array[i].score > array[length-1].score)
        {
            detectionSwap(array + i, array + pivot);
            pivot++;
        }
    }
    // swap the pivot value into position
    detectionSwap(array + pivot, array + length - 1);
    return pivot;
}

static void detectionsQuickSort(ObjectDetection* array, const int count)
{
#if DEBUG == 1
    assert(array != NULL);
#endif
	int pivot = 0;
    if (count < 2)
        return;
    pivot = detectionsPartition(array, count);
    detectionsQuickSort(array, pivot);
    pivot += 1;//skips pivot slot
    detectionsQuickSort(array + pivot, count - pivot);
}

static int overlappingArea(const Rect rect1, const Rect rect2)
{
    //a and b are expected to be tuples of the type (x1, y1, x2, y2)
    //code adapted from http://visiongrader.sf.net
	const int maxX = Pedestrian_MIN(rect1.maxX, rect2.maxX);
	const int minX = Pedestrian_MAX(rect1.minX, rect2.minX);
	const int maxY = Pedestrian_MIN(rect1.maxY, rect2.maxY);
	const int minY = Pedestrian_MAX(rect1.minY, rect2.minY);
	const int width = maxX - minX;
	const int height = maxY - minY;
    if (width < 0 || height < 0)
    {
        return 0;
    }
    else
    {
        return width * height;
    }
}

static int unionArea(const Rect rect1, const Rect rect2)
{
    // a and b are expected to be tuples of the type (x1, y1, x2, y2)
    // code adapted from http://visiongrader.sf.net
    const int area1 = rectangleArea(rect1);
    const int area2 = rectangleArea(rect2);
    const int inter = overlappingArea(rect1, rect2);
    return area1 + area2 - inter;
}

/*
I/O:	Name		                              Type	                             Size				Content
[in]  detection1                            const ObjectDetection   1                the first detection.
[in]  detection2                            const ObjectDetection   1                the second detection.
[in]	method		                          const int	                       1				 defines overlap criterion used:  1<dollar> 2<pascal>"
                                                                                                                 dollar: The non maximal suppression is based on greedy variant,
                                                                                                                 see Section 1.2.1 P. Dollar, Integral Channel Features - Addendum, 2009.
                                                                                                                 pascal: PASCALVOC overlap over union.
Realized function:
    + Compute the overlap between two detections, using the P. Dollar overlap criterion!
    + this is _not_ the PASCAL VOC overlap criterion.
*/
static float computeOverlap(const ObjectDetection detection1, const ObjectDetection detection2, const int method)
{
	int unionAreaValue = 0;
	int intersectionArea = 0;
	int area1 = 0;
	int area2 = 0;
	int minArea = 0;
	Rect rect1;
	Rect rect2;
	rect1.minX = detection1.x;
	rect1.minY = detection1.y;
	rect1.maxX = detection1.x + detection1.width;
	rect1.maxY = detection1.y + detection1.height;
	rect2.minX = detection2.x;
	rect2.minY = detection2.y;
	rect2.maxX = detection2.x + detection2.width;
	rect2.maxY = detection2.y + detection2.height;
    if (method == 1)
    {
        intersectionArea = overlappingArea(rect1, rect2);
		area1 = detection1.width * detection1.height;
		area2 = detection2.width * detection2.height;
		minArea = Pedestrian_MIN(area1, area2);
        return (float)intersectionArea / minArea;
    }
    else if (method == 2)
    {
        unionAreaValue = unionArea(rect1, rect2);
        intersectionArea = overlappingArea(rect1, rect2);
        return (float)intersectionArea / unionAreaValue;
    }
    else
    {
        return 0;
    }
}

void greedyNonMaximalSuppression(const DetectorOutputObject* candidateDetections, const float minimalOverlapThreshold, DetectorOutputObject* finalDetections)
{
#if DEBUG == 1
    assert(candidateDetections != NULL);
    assert(finalDetections != NULL);
#endif
	const int detectionCount = candidateDetections->outputObjectCount;
    int index = 0;
    int secondIndex = 0;
    int finalDetectionCount = 0;
	float overlap = 0;

    unsigned char *detectionsIserase = (unsigned char *)my_calloc(detectionCount, sizeof(unsigned char));
    memset(detectionsIserase, 1, detectionCount *sizeof(unsigned char));

	detectionsQuickSort(candidateDetections->outputObjectDetections, detectionCount);

    for(index = 0; index < detectionCount; index++)
    {
        if(detectionsIserase[index] != 0)
        {
            finalDetections->outputObjectDetections[finalDetectionCount] = candidateDetections->outputObjectDetections[index];
            finalDetectionCount++;
            for(secondIndex = index + 1; secondIndex < detectionCount; secondIndex++)
            {
                if(detectionsIserase[secondIndex] != 0)
                {
                    overlap = computeOverlap(candidateDetections->outputObjectDetections[index], candidateDetections->outputObjectDetections[secondIndex], 1);

                    if(overlap > minimalOverlapThreshold)
                    {
                        // this detection seems to overlap too much, we should remove it
                        detectionsIserase[secondIndex] = 0;
                    }
                }
            }
        }
    }
	finalDetections->outputObjectCount = finalDetectionCount;
    my_free(detectionsIserase);
    detectionsIserase = NULL;
}
