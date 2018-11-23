#ifndef TEST_UTILITY_H
#define TEST_UTILITY_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <fstream>
#include <string>

extern "C"
{
#include "baseAlgorithm/image_data_structure.h"
}
void logMat(const char* fileName, const cv::Mat image);

void logImageU(const char* fileName, const unsigned char *img, const int width, const int height);

void logPoint(const char* fileName, const Point *point, const int count);
void logFPoint(const char* fileName, const FloatPoint *cornerPoint, const int count);

int matWrite(const cv::Mat &img, const std::string &fileName);

int readImage(const char *fileName, unsigned char *image);

#endif // TEST_UTILITY_H
