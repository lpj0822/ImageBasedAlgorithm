#ifndef LK_OPTICALFLOW_H
#define LK_OPTICALFLOW_H

#include "opticalflow_data_struct.h"
#include "lk_trajectory.h"

#define MAX_THREAD_NUM 4

//金字塔层数
#define K_IT 3
#define MAX_CORNERS 10000
#define  wx  (5)
#define  wy  (5)
#define  OPTICAL_WINDOW_COUNT  124 //(2*wx+1)*(2*wy+1) extend four times

#define XUNHUAN_K_IT  (K_IT -1)

typedef struct LK_INNER
{
    //金字塔每层的长宽
    Size * SizeOfPyr;

    //强角点的信息
    CornerPoint * cornersA;
    CornerPoint * cornersB;
    CornerPoint * cornersC;
    CornerPoint * ctemp;
    CornerPoint * Matched_Precorners;
    CornerPoint * Matched_corners;

    //金字塔各层图像及指针
    unsigned char *const_prePyr[K_IT] ;     //fang
    unsigned char *const_thisPyr[K_IT];    //fang

    //角点找到信息
    unsigned short *features_found;
    unsigned short *features_found_init;
    unsigned short *m_Index;
    Trajectory *pTrajecySet;
    int nTrajecyNum;
    //是否得到前一帧数据
    int nGetPreImg;
    int corner_num;
    int matched_corner_num ;

    unsigned char  **prePyr;
    unsigned char  **thisPyr;
    CornerPoint *choose_point;
    CornerPoint **LastTrajCorn; //上一次所有估计的Corner
    BUFFER_MGR mem;
}lk_inner;


int OpticalAnalyze(unsigned char *image,int w, int h);

void mvGetLKTrajecy(Trajectory **pLkTrajecy, int *pTrajecNum);


#endif // LK_OPTICALFLOW_H
