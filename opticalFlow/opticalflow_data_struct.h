#ifndef OPTICALFLOW_DATA_STRUCT_H
#define OPTICALFLOW_DATA_STRUCT_H

#include "baseAlgorithm/image_data_structure.h"

typedef struct CornerPoint
{
    float x;
    float y;
    unsigned char bPreTrack;
    FloatPoint earlyestPoint; //最早被匹配的点
    int nMatchedDir; //匹配方向
    int nLength;
    int nLeft;
}CornerPoint;

typedef struct _BUFFER_MGR_
{
    int total;
    int curr;
    unsigned char *start;
    unsigned char *end;
}BUFFER_MGR;

#endif // OPTICALFLOW_DATA_STRUCT_H
