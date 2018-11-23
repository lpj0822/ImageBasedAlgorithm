#include "test_opticalflow.h"
#include <vector>

extern "C"
{
#include "opticalFlow/lk_opticalflow.h"
}

#include "testprocesstime.h"
#include "test_utility.h"

static void processImage(const char* imagePath)
{
    int i = 0;
    const int width = 720;
    const int height = 480;
    unsigned int frameIndex = 0;
    char flieName[1024] = "";
    unsigned char *image = NULL;
    image = (unsigned char *)malloc(width * height * sizeof(unsigned char));
    memset(image, 0, width * height * sizeof(unsigned char));
    for(frameIndex = 0; frameIndex < 3; frameIndex += 1)
    {
        sprintf(flieName, "%s/optical_img%d.dat", imagePath, (int)frameIndex);
        printf("image: %s\n", flieName);
        if(readImage(flieName, image) >= 0)
        {
            double ts = (double)cvGetTickCount();
            OpticalAnalyze(image, 720, 480);
            printf("OpticalAnalyze_take_time:%0.3f\n",(cvGetTickCount() - ts)/(cvGetTickFrequency() * 1000.0f));

            Trajectory *pLKTrajecy = NULL;
            int nTrayjecyNum = 0;

            mvGetLKTrajecy(&pLKTrajecy, &nTrayjecyNum);

            for (i = 0 ;  i < nTrayjecyNum; i += 1)
            {
                Trajectory  *pTrajecy = pLKTrajecy + i;
                CornerPoint * pTrackConer = pTrajecy->point + ((pTrajecy->PoitNum - 1) & MAX_TRAJECY_POINT_BIT);
                //printf("Optical_value point %d: %0.3f %0.3f\n", i, pTrackConer->x, pTrackConer->y);
            }
        }
    }
    free(image);
    image = NULL;
}

static int processVideo(const char* videoPath)
{
    int i,j;
    int nId=0;

    cv::VideoCapture video;
    cv::Mat srcFrame;
    video.open(videoPath);
    if(!video.isOpened())
    {
        return -1;
    }

    while (nId < 3)
    {
        if(!video.read(srcFrame))
        {
            break;
        }
        cv::Mat frame = srcFrame(cv::Rect(720, 480, 720, 480));
        printf("******************************frame:%d*********************************\n", nId);

        cv::Mat gray;
        std::stringstream imagePath;

        cv::cvtColor(frame, gray, CV_RGB2GRAY);

        //imagePath << "optical_img" << nId << ".dat";
        //matWrite(gray, imagePath.str());

        double ts = (double)cvGetTickCount();
        OpticalAnalyze(gray.data, gray.cols, gray.rows);
        printf("OpticalAnalyze_take_time:%0.3f\n",(cvGetTickCount() - ts)/(cvGetTickFrequency() * 1000.0f));

        Trajectory *pLKTrajecy = NULL;
        int nTrayjecyNum = 0;

        mvGetLKTrajecy(&pLKTrajecy, &nTrayjecyNum);

        int h = gray.rows;
        int w = gray.cols;

        //绘制匹配点结果
        cv:: Mat CombaredImage(h, w ,CV_8UC3);
        std::vector<cv::Mat> mv;
        mv.push_back(gray);
        mv.push_back(gray);
        mv.push_back(gray);

        cv::merge(mv, CombaredImage(cv::Rect(0 ,0, w, h)));

        //绘制轨迹
        for (i = 0 ;  i < nTrayjecyNum; i += 4)
        {
            Trajectory  *pTrajecy = pLKTrajecy + i;
            CvScalar color = CV_RGB(rand()%255,rand()%255,rand()%255);

            for ( j = 0 ; j < pTrajecy->PoitNum - 1; j++ )
            {
                CornerPoint * pTrackConer= pTrajecy->point + (j & MAX_TRAJECY_POINT_BIT);
                CornerPoint * pNextTrackConer = pTrajecy->point + ((j + 1)&MAX_TRAJECY_POINT_BIT);

                float fds = PNorm(pTrackConer , pNextTrackConer);

                cv::line(CombaredImage, cv::Point(pTrackConer->x,pTrackConer->y),
                    cv::Point(pNextTrackConer->x,pNextTrackConer->y), color);

            }
            CornerPoint * pTrackConer = pTrajecy->point + ((pTrajecy->PoitNum - 1) & MAX_TRAJECY_POINT_BIT);
            printf("Optical_value point %d: %0.3f %0.3f\n", i, pTrackConer->x, pTrackConer->y);
            cv::circle(CombaredImage, cv::Point(pTrackConer->x, pTrackConer->y), 3, color);
        }

        char NumLab[10];
        sprintf(NumLab,"%d", nId);
        cv::putText(CombaredImage, NumLab, cv::Point(130,30), CV_FONT_HERSHEY_COMPLEX, 1.0f, cv::Scalar(0,0,255), 1);
        cv::imshow("CombaredImage1", CombaredImage);
        if(cv::waitKey(1) == 27)
            break;

        nId++;
    }
    system("pause");
    video.release();
    cvDestroyAllWindows();
    return 0;
}

void testOpticalFlow()
{
    const char* videoPath = "C:/Users/lpj/Desktop/QtProject/data/Wuxi_Perdestrain_Test_Video/YSA201512090078.avi";
    processVideo(videoPath);
}
