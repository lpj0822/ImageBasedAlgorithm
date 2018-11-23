#ifndef LK_TRAJECTORY_H
#define LK_TRAJECTORY_H

#include <math.h>
#include "opticalflow_data_struct.h"

#define  MAX_TRAJECY_NUM 1024
#define  MAX_POINT_NUM_PER_TRAJECY 64
#define  MAX_TRAJECY_POINT_BIT 63  // MAX_POINT_NUM_PER_TRAJECY必须是2的次数幂，且MAX_TRAJECY_BIT == MAX_POINT_NUM_PER_TRAJECY-1
#define  TRAJECY_INVALID -1

#define Norm(A,B) sqrtf( (A.x -B.x)*(A.x -B.x) +  (A.y -B.y)*(A.y -B.y))
#define PNorm(A,B) sqrtf( (A->x -B->x)*(A->x -B->x) +  (A->y -B->y)*(A->y -B->y))

typedef struct Trajectory
{
    CornerPoint *point;
    int PoitNum;
    int ntrackId;
    int nDir;
    int nLeft;
}Trajectory;

void  updataTrajectory(Trajectory *pTrajecy, CornerPoint *pConner);
void  addTrajectory(Trajectory  *pTrajecy, int listId, CornerPoint *pConner);
void  DelTrajectory(Trajectory  *pTrajecy);
void  CopyTrajectory(Trajectory  *DstpTrajecy, Trajectory  *SrcpTrajecy);

#endif // LK_TRAJECTORY_H
