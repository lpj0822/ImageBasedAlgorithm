#include "test_image_resize.h"

extern "C"
{
#include "imageResize/image_resize.h"
}

#include "testprocesstime.h"
#include "test_utility.h"

#define RGB 0

void testImageResize()
{
    const int resizeWidth = 160;
    const int resizeHeight = 120;
    cv::Mat matSrc;
    cv::Mat matDst;
    cv::Mat opencvDst;
#if RGB == 0
    matSrc = cv::imread("F:/github/ImageBasedAlgorithm/testData/person_0.png", 0);
    if(matSrc.empty())
    {
        printf("image empty!\n");
        return;
    }
    matDst = cv::Mat(cv::Size(resizeWidth, resizeHeight), matSrc.type(), cv::Scalar::all(0));
    opencvDst = cv::Mat(matDst.size(), matSrc.type(), cv::Scalar::all(0));

    grayImageResizeOfNeighborInterpolation(matSrc.data, matSrc.cols, matSrc.rows, resizeWidth, resizeHeight, matDst.data);
    cv::imwrite("dst.jpg", matDst);
    logMat("dst.txt", matDst);

    computeShrinkedImage(matSrc.data, matSrc.cols, matSrc.rows, resizeWidth, resizeHeight, matDst.data);
    cv::imwrite("dst1.jpg", matDst);
    logMat("dst1.txt", matDst);

    cv::resize(matSrc, opencvDst, opencvDst.size(), 0, 0, cv::INTER_NEAREST);
    cv::imwrite("opencv_dst.jpg", opencvDst);
    logMat("opencv_dst.txt", opencvDst);
#else
    cv::Mat rgbMat;
    ElementRGB *rgbImage = NULL;
    ElementRGB *dstRgbImage = NULL;

    matSrc = cv::imread("F:/github/ImageBasedAlgorithm/testData/person_0.png", 1);
    if(matSrc.empty())
    {
        printf("image empty!\n");
        return;
    }
    cv::cvtColor(matSrc, rgbMat, cv::COLOR_BGR2RGB);
    matDst = cv::Mat(cv::Size(resizeWidth, resizeHeight), matSrc.type(), cv::Scalar::all(0));
    opencvDst = cv::Mat(matDst.size(), matSrc.type(), cv::Scalar::all(0));

    rgbImage = (ElementRGB*)rgbMat.data;
    dstRgbImage = (ElementRGB*)matDst.data;
    rgbImageResizeOfNeighborInterpolation(rgbImage, rgbMat.cols, rgbMat.rows, resizeWidth, resizeHeight, dstRgbImage);
    cv::cvtColor(matDst, matDst, cv::COLOR_RGB2BGR);
    cv::imwrite("dst.jpg", matDst);
    logMat("dst.txt", matDst);

    cv::resize(matSrc, opencvDst, opencvDst.size(), 0, 0, cv::INTER_NEAREST);
    cv::imwrite("opencv_dst.jpg", opencvDst);
    logMat("opencv_dst.txt", opencvDst);
#endif
    printf("image resize end!\n");
}
