#include "lk_trajectory.h"
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include "helpers/utility.h"

void  addTrajectory(Trajectory  *pTrajecy, int listId, CornerPoint *pConner)
{
    assert(0 == pTrajecy->PoitNum);
    if(TRAJECY_INVALID == pTrajecy->ntrackId)//如果新的位置没有id号的话，就用当前的轨迹数目NUM作为其Id号
    {
        pTrajecy->ntrackId = listId;
    }
    pTrajecy->point[pTrajecy->PoitNum & MAX_TRAJECY_POINT_BIT] = *pConner;
    pTrajecy->nLeft = pConner->nLeft;
    pTrajecy->PoitNum++;
}

void  DelTrajectory(Trajectory  *pTrajecy)
{
    pTrajecy->PoitNum = 0;
}

void  CopyTrajectory(Trajectory  *DstpTrajecy, Trajectory  *SrcpTrajecy)
{
    int tempid = 0;
    memcpy(DstpTrajecy->point,SrcpTrajecy->point,sizeof(CornerPoint)*MAX_POINT_NUM_PER_TRAJECY);
    DstpTrajecy->PoitNum = SrcpTrajecy->PoitNum;
    DstpTrajecy->nLeft = SrcpTrajecy->nLeft;
    tempid = DstpTrajecy->ntrackId;
    DstpTrajecy->ntrackId = SrcpTrajecy->ntrackId;
    SrcpTrajecy->ntrackId = tempid;
}

void  updataTrajectory(Trajectory *pTrajecy, CornerPoint *pConner)
{
    assert( TRAJECY_INVALID != pTrajecy->ntrackId && pTrajecy->PoitNum );
    pTrajecy->point[pTrajecy->PoitNum & MAX_TRAJECY_POINT_BIT] = *pConner;
    pTrajecy->PoitNum++;
}
