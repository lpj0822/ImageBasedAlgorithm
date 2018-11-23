#include "test_corner_detection.h"

extern "C"
{
#include "features2d/fast_corner_detection.h"
}

#include "testprocesstime.h"
#include "test_utility.h"

static int processVideo(const char* videoPath)
{
    int i = 0;
    int nId = 0;

    cv::VideoCapture video;
    cv::Mat frame1;
    cv::Mat gray;
    int cornerCount = 0;
    FloatPoint fastCorner[MAX_FAST_CORNERS] = {0};
    video.open(videoPath);
    if(!video.isOpened())
    {
        return -1;
    }

    while (video.read(frame1))
    {
        cv::Mat frame = frame1(cv::Rect(720, 480, 720, 480));
        printf("******************************%d*********************************\n", nId);

        cvtColor(frame, gray, CV_RGB2GRAY);

        double ts = (double)cvGetTickCount();
        fast9_16(gray.data, gray.cols, gray.rows, 10, 0, gray.cols, 0, gray.rows, &cornerCount, fastCorner);
        printf("fast_corner_take_time:%0.3fms\n",(cvGetTickCount() - ts)/(cvGetTickFrequency() * 1000.0f));

        logFPoint("fast_corner.txt", fastCorner, cornerCount);

        //draw
        for (i = 0 ;  i < cornerCount; i++)
        {
            cv::circle(frame, cv::Point(fastCorner[i].x, fastCorner[i].y), 3, cv::Scalar(0, 0, 255));
        }

        char NumLab[10];
        sprintf(NumLab,"%d", nId);
        cv::putText(frame, NumLab, cv::Point(130,30), CV_FONT_HERSHEY_COMPLEX, 1.0f, cv::Scalar(0,0,255), 1);
        cv::imshow("src", frame);
        if(cv::waitKey(0) == 27)
            break;

        nId++;
    }
    video.release();
    cvDestroyAllWindows();
    return 0;
}

void testCornerDetection()
{
    const char* videoPath = "C:/Users/lpj/Desktop/QtProject/data/Wuxi_Perdestrain_Test_Video/YSA201512090078.avi";
    processVideo(videoPath);
}
