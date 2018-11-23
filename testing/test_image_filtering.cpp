#include "test_image_filtering.h"

extern "C"
{
#include "imageFiltering/base_image_filter.h"
}

#include "testprocesstime.h"
#include "test_utility.h"

void testImageFiltering()
{
    cv::Mat matSrc;
    cv::Mat matDst;
    cv::Mat opencvDst;
    float kernel[3] = {1.0f, 2.0f, 1.0f};

    matSrc = cv::imread("F:/github/ImageBasedAlgorithm/testData/person_0.png", 0);
    if(matSrc.empty())
    {
        printf("image empty!\n");
        return;
    }

    matDst = cv::Mat(cv::Size(matSrc.cols, matSrc.rows), CV_8UC1);
    opencvDst = cv::Mat(matDst.size(), CV_8UC1);

    gaussianSmoothGray(matSrc.data, matSrc.cols, matSrc.rows, kernel, 3, matDst.data);
    logMat("gaussian.txt", matDst);

    cv::GaussianBlur(matSrc, opencvDst, cv::Size(3, 3), 0, 0);
    logMat("opencv_g.txt", opencvDst);

    cv::imshow("my gaussian", matDst);
    cv::imshow("opencv", opencvDst);
    cv::imshow("src", matSrc);
    cv::waitKey(0);

    printf("image filtering end!\n");
}
