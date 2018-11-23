#include "test_color_converting.h"

extern "C"
{
#include "colorConverting/color_converting.h"
}

#include "testprocesstime.h"
#include "test_utility.h"

void testColorConverting()
{
    cv::Mat matSrc;
    cv::Mat matDst;
    cv::Mat opencvDst;
    cv::Mat rgbMat;
    ElementRGB *rgbImage = NULL;
    ElementLAB *dstRgbImage = NULL;

    matSrc = cv::imread("F:/github/ImageBasedAlgorithm/testData/person_0.png", 1);
    if(matSrc.empty())
    {
        printf("image empty!\n");
        return;
    }

    cv::cvtColor(matSrc, rgbMat, cv::COLOR_BGR2RGB);
    matDst = cv::Mat(cv::Size(rgbMat.cols, rgbMat.rows), CV_8UC3);
    opencvDst = cv::Mat(matDst.size(), CV_8UC3);

    rgbImage = (ElementRGB*)rgbMat.data;
    dstRgbImage = (ElementLAB*)matDst.data;
    rgbToLab(rgbImage, rgbMat.cols, rgbMat.rows, dstRgbImage);
    logMat("lab.txt", matDst);

    cv::cvtColor(rgbMat, opencvDst, cv::COLOR_RGB2Lab);
    logMat("opencv_lab.txt", opencvDst);

    printf("image color converting end!\n");
}
