#include "lk_opticalflow.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "helpers/utility.h"

static lk_inner *g_pLkInner = NULL;

#ifndef CHECK_RET_VAL
#define CHECK_RET_VAL(chk, val)  if ((chk)) {return (val);}
#endif

#define CHECK_FREE(addr)          if((addr)) {free((addr)); (addr) = 0;}
#define SIZE_ALIGN(SIZE, align) (((SIZE) + (align - 1)) & ((unsigned int)(-align))) // align computation

unsigned char* alloc_buff(BUFFER_MGR *buff, int size)
{
    unsigned char *pos = 0;
    int nAlign = 16;
    CHECK_RET_VAL(0 == buff, 0);
    CHECK_RET_VAL((buff->curr + SIZE_ALIGN(size, nAlign)) > buff->total, 0);
    pos = buff->start + buff->curr;
    buff->curr += SIZE_ALIGN(size, nAlign);
    memset(pos, 0, SIZE_ALIGN(size, nAlign));

    return pos;
}

//申请金字塔各层图像空间及指针
void mvLKinit(int wid, int hgt)
{
    int i = 0 ;
    int memSize,memMax;
    int sum;
    Trajectory *pTrajecy = NULL;
    if(g_pLkInner != NULL)
    {
        return;
    }

    g_pLkInner = (lk_inner *) malloc(sizeof(lk_inner));

    memSize = wid * hgt;

    memMax = K_IT * sizeof(Size) + MAX_CORNERS * sizeof(CornerPoint) * 6 + (K_IT + 1) * memSize\
            + sizeof(unsigned short) * MAX_CORNERS * 3 + sizeof(CornerPoint) * MAX_CORNERS + memSize + \
            + MAX_TRAJECY_NUM * sizeof(Trajectory) + MAX_TRAJECY_NUM * MAX_POINT_NUM_PER_TRAJECY * sizeof(CornerPoint) + \
            + MAX_TRAJECY_NUM * sizeof(CornerPoint*) + 1024 * 4;


    //开辟内存
    g_pLkInner->mem.start = (unsigned char*) malloc(memMax);
    memset(g_pLkInner->mem.start, 0, memMax);
    g_pLkInner->mem.curr  = 0;
    g_pLkInner->mem.total = memMax;
    g_pLkInner->mem.end   = g_pLkInner->mem.start + g_pLkInner->mem.total;

    //金字塔每层的长宽
    g_pLkInner->SizeOfPyr=(Size *)alloc_buff(&g_pLkInner->mem, K_IT * sizeof(Size));//fang
    //g_pLkInner
    g_pLkInner->cornersA = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS * sizeof(CornerPoint));
    g_pLkInner->cornersB = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS * sizeof(CornerPoint));
    g_pLkInner->cornersC = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS * sizeof(CornerPoint));
    g_pLkInner->ctemp = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS * sizeof(CornerPoint));
    g_pLkInner->Matched_Precorners = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS*sizeof(CornerPoint));
    g_pLkInner->Matched_corners = (CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_CORNERS*sizeof(CornerPoint));

    //set every floor size
    g_pLkInner->SizeOfPyr[0].width = wid;
    g_pLkInner->SizeOfPyr[0].height = hgt;

    for( i = 1; i < K_IT ; i++ )//fang
    {
        g_pLkInner->SizeOfPyr[i].width = g_pLkInner->SizeOfPyr[i-1].width >> 1;
        g_pLkInner->SizeOfPyr[i].height = g_pLkInner->SizeOfPyr[i-1].height >> 1;
    }

    //构建金子塔指针
    for( i = 0 ; i < K_IT ; i++)
    {
        //求图像大小
        sum = g_pLkInner->SizeOfPyr[i].width * g_pLkInner->SizeOfPyr[i].height;

        g_pLkInner->const_thisPyr[i] = (unsigned char *)alloc_buff(&g_pLkInner->mem, sum * sizeof(unsigned char));
        memset(g_pLkInner->const_thisPyr[i],0, sum * sizeof(unsigned char));

        g_pLkInner->const_prePyr[i] = (unsigned char *)alloc_buff(&g_pLkInner->mem, sum * sizeof(unsigned char) );
    }

    g_pLkInner->features_found_init = (unsigned short *)alloc_buff(&g_pLkInner->mem, sizeof(unsigned short) * MAX_CORNERS );

    for (i = 0 ; i< MAX_CORNERS ;i++ )
    {
        g_pLkInner->features_found_init[i] = 1;
    }

    g_pLkInner->features_found = (unsigned short *)alloc_buff(&g_pLkInner->mem,sizeof(unsigned short) * MAX_CORNERS );
    g_pLkInner->m_Index = (unsigned short *)alloc_buff(&g_pLkInner->mem,sizeof(unsigned short) * MAX_CORNERS );
    g_pLkInner->choose_point= (CornerPoint *)alloc_buff(&g_pLkInner->mem, sizeof(CornerPoint) * MAX_CORNERS );
    g_pLkInner->pTrajecySet = (Trajectory *)alloc_buff(&g_pLkInner->mem, MAX_TRAJECY_NUM * sizeof(Trajectory));

    for (i = 0 ; i < MAX_TRAJECY_NUM; i++)
    {
        pTrajecy = g_pLkInner->pTrajecySet + i;
        pTrajecy->point =(CornerPoint *)alloc_buff(&g_pLkInner->mem, MAX_POINT_NUM_PER_TRAJECY * sizeof(CornerPoint));
        pTrajecy->ntrackId = TRAJECY_INVALID;
        pTrajecy->PoitNum = 0;
    }
    g_pLkInner->LastTrajCorn = (CornerPoint **)alloc_buff(&g_pLkInner->mem, MAX_TRAJECY_NUM * sizeof(CornerPoint*));
    g_pLkInner->nTrajecyNum = 0 ;

    g_pLkInner->nGetPreImg = 0;
    g_pLkInner->corner_num = 0;
    g_pLkInner->matched_corner_num = 0;
    g_pLkInner->prePyr = &g_pLkInner->const_prePyr[0];
    g_pLkInner->thisPyr= &g_pLkInner->const_thisPyr[0];
}

/************************************************************************/
/* 计算上层金字塔
[in]   lowerImg           下一层金字塔图像   unsigned char *
[in]   lowerImgSize      size of lowerImg     bsdsize *
[in]   upperImgSize      size of upperImg    bsdsize *
[out] upperImg           上一层金字塔图像   unsigned char *
*/
/************************************************************************/
unsigned short CalSubPyr(const unsigned char* lowerImg, unsigned char* upperImg, Size *lowerImgSize,\
                 Size *upperImgSize, int nYThresh)
{
    int lowerImgWidth = lowerImgSize->width;
    int upperImgWidth = upperImgSize->width;
    unsigned short U16 = 0;
    unsigned short U8 = 0;
    unsigned short U4 = 0;
    // unsigned short U0 = 0;
    int i , j;
    int pos;
    nYThresh =  (nYThresh >>1);
    for (j = nYThresh; j < upperImgSize->height-1; j++)
    {
        for(i=0;i<upperImgWidth-1;i++)
        {
            //8邻域 4个角
            pos = (j * lowerImgWidth + i) << 1;//(x,y)<->(2x,2y)
            U16 = lowerImg[pos]+
                lowerImg[pos + 2]+
                lowerImg[pos + (lowerImgWidth << 1)]+
                lowerImg[pos + (lowerImgWidth << 1) + 2];

            U8 = lowerImg[pos + 1]+
                lowerImg[pos + (lowerImgWidth << 1) + 1]+
                lowerImg[pos + lowerImgWidth]+
                lowerImg[pos + lowerImgWidth + 2];

            U4 = lowerImg[pos + lowerImgWidth + 1];

            *(upperImg + j*upperImgWidth + i)= (U16 + (U8 << 1) + (U4 << 2)) >> 4;

        }
    }

    return 1;

}

unsigned short myCreatePyr(unsigned char**Pyr, int nYThresh)
{
    int i;
    //逐层建立金字塔
    for(i = 0;i < K_IT - 1; i++)
    {
        nYThresh = ( nYThresh >> i);

        CalSubPyr(Pyr[i], Pyr[i + 1], g_pLkInner->SizeOfPyr + i,
                g_pLkInner->SizeOfPyr + i + 1, nYThresh);

    }
    return 1;

}


void oast9_16(const unsigned char* im, int xsize, int ysize, int b, \
              int* num_corners, CornerPoint *pXyCorner,int nStart_y,int nEnd_y,int nStart_x,int nEnd_x)
{
    int total = 0;
    /*int stepx =MAX((nEnd_x-nStart_x)/240,1);
    int stepy =MAX((nEnd_y-nStart_y)/80,1);*/
    int stepx =3;
    int stepy =2;

    register int x, y;
    register int offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, \
        offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
    register int width;
    int cb ;
    int c_b;
    int start_y,End_y;

    start_y = nStart_y;
    End_y = nEnd_y;

    offset0=(-3)+(0)*xsize;
    offset1=(-3)+(-1)*xsize;
    offset2=(-2)+(-2)*xsize;
    offset3=(-1)+(-3)*xsize;
    offset4=(0)+(-3)*xsize;
    offset5=(1)+(-3)*xsize;
    offset6=(2)+(-2)*xsize;
    offset7=(3)+(-1)*xsize;
    offset8=(3)+(0)*xsize;
    offset9=(3)+(1)*xsize;
    offset10=(2)+(2)*xsize;
    offset11=(1)+(3)*xsize;
    offset12=(0)+(3)*xsize;
    offset13=(-1)+(3)*xsize;
    offset14=(-2)+(2)*xsize;
    offset15=(-3)+(1)*xsize;

    width = xsize;

    for(y = start_y + stepy; y < End_y; y++)
    {
        x = nStart_x;
        while(1)
        {
            x = x + stepx;

            if(x >nEnd_x)
                break;
            else
            {
                register const unsigned char* const p = im + y * width + x;

                cb = *p + b;
                c_b = *p - b;//b为设定的阈值
                if(p[offset0] > cb)
                    if(p[offset2] > cb)
                        if(p[offset4] > cb)
                            if(p[offset5] > cb)
                                if(p[offset7] > cb)
                                    if(p[offset3] > cb)
                                        if(p[offset1] > cb)
                                            if(p[offset6] > cb)
                                                if(p[offset8] > cb)
                                                {}
                                                else
                                                    if(p[offset15] > cb)
                                                    {}
                                                    else
                                                        continue;
                                            else
                                                if(p[offset13] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset8] > cb)
                                                if(p[offset9] > cb)
                                                    if(p[offset10] > cb)
                                                        if(p[offset6] > cb)
                                                        {}
                                                        else
                                                            if(p[offset11] > cb)
                                                                if(p[offset12] > cb)
                                                                    if(p[offset13] > cb)
                                                                        if(p[offset14] > cb)
                                                                            if(p[offset15] > cb)
                                                                            {}
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] > cb)
                                            if(p[offset11] > cb)
                                                if(p[offset12] > cb)
                                                    if(p[offset8] > cb)
                                                        if(p[offset9] > cb)
                                                            if(p[offset6] > cb)
                                                            {}
                                                            else
                                                                if(p[offset13] > cb)
                                                                    if(p[offset14] > cb)
                                                                        if(p[offset15] > cb)
                                                                        {}
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                        else
                                                            if(p[offset1] > cb)
                                                                if(p[offset13] > cb)
                                                                    if(p[offset14] > cb)
                                                                        if(p[offset15] > cb)
                                                                        {}
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset1] > cb)
                                                            if(p[offset13] > cb)
                                                                if(p[offset14] > cb)
                                                                    if(p[offset15] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else if(p[offset7] < c_b)
                                    if(p[offset14] > cb)
                                        if(p[offset15] > cb)
                                            if(p[offset1] > cb)
                                                if(p[offset3] > cb)
                                                    if(p[offset6] > cb)
                                                    {}
                                                    else
                                                        if(p[offset13] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] > cb)
                                                        if(p[offset11] > cb)
                                                            if(p[offset12] > cb)
                                                                if(p[offset13] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset8] > cb)
                                                    if(p[offset9] > cb)
                                                        if(p[offset10] > cb)
                                                            if(p[offset11] > cb)
                                                                if(p[offset12] > cb)
                                                                    if(p[offset13] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else if(p[offset14] < c_b)
                                        if(p[offset8] < c_b)
                                            if(p[offset9] < c_b)
                                                if(p[offset10] < c_b)
                                                    if(p[offset11] < c_b)
                                                        if(p[offset12] < c_b)
                                                            if(p[offset13] < c_b)
                                                                if(p[offset6] < c_b)
                                                                {}
                                                                else
                                                                    if(p[offset15] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    if(p[offset14] > cb)
                                        if(p[offset15] > cb)
                                            if(p[offset1] > cb)
                                                if(p[offset3] > cb)
                                                    if(p[offset6] > cb)
                                                    {}
                                                    else
                                                        if(p[offset13] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] > cb)
                                                        if(p[offset11] > cb)
                                                            if(p[offset12] > cb)
                                                                if(p[offset13] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset8] > cb)
                                                    if(p[offset9] > cb)
                                                        if(p[offset10] > cb)
                                                            if(p[offset11] > cb)
                                                                if(p[offset12] > cb)
                                                                    if(p[offset13] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else if(p[offset5] < c_b)
                                if(p[offset12] > cb)
                                    if(p[offset13] > cb)
                                        if(p[offset14] > cb)
                                            if(p[offset15] > cb)
                                                if(p[offset1] > cb)
                                                    if(p[offset3] > cb)
                                                    {}
                                                    else
                                                        if(p[offset10] > cb)
                                                            if(p[offset11] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset8] > cb)
                                                        if(p[offset9] > cb)
                                                            if(p[offset10] > cb)
                                                                if(p[offset11] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset6] > cb)
                                                    if(p[offset7] > cb)
                                                        if(p[offset8] > cb)
                                                            if(p[offset9] > cb)
                                                                if(p[offset10] > cb)
                                                                    if(p[offset11] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(p[offset12] < c_b)
                                    if(p[offset7] < c_b)
                                        if(p[offset8] < c_b)
                                            if(p[offset9] < c_b)
                                                if(p[offset10] < c_b)
                                                    if(p[offset11] < c_b)
                                                        if(p[offset13] < c_b)
                                                            if(p[offset6] < c_b)
                                                            {}
                                                            else
                                                                if(p[offset14] < c_b)
                                                                    if(p[offset15] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                if(p[offset12] > cb)
                                    if(p[offset13] > cb)
                                        if(p[offset14] > cb)
                                            if(p[offset15] > cb)
                                                if(p[offset1] > cb)
                                                    if(p[offset3] > cb)
                                                    {}
                                                    else
                                                        if(p[offset10] > cb)
                                                            if(p[offset11] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset8] > cb)
                                                        if(p[offset9] > cb)
                                                            if(p[offset10] > cb)
                                                                if(p[offset11] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset6] > cb)
                                                    if(p[offset7] > cb)
                                                        if(p[offset8] > cb)
                                                            if(p[offset9] > cb)
                                                                if(p[offset10] > cb)
                                                                    if(p[offset11] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(p[offset12] < c_b)
                                    if(p[offset7] < c_b)
                                        if(p[offset8] < c_b)
                                            if(p[offset9] < c_b)
                                                if(p[offset10] < c_b)
                                                    if(p[offset11] < c_b)
                                                        if(p[offset13] < c_b)
                                                            if(p[offset14] < c_b)
                                                                if(p[offset6] < c_b)
                                                                {}
                                                                else
                                                                    if(p[offset15] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else if(p[offset4] < c_b)
                            if(p[offset11] > cb)
                                if(p[offset12] > cb)
                                    if(p[offset13] > cb)
                                        if(p[offset10] > cb)
                                            if(p[offset14] > cb)
                                                if(p[offset15] > cb)
                                                    if(p[offset1] > cb)
                                                    {}
                                                    else
                                                        if(p[offset8] > cb)
                                                            if(p[offset9] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset6] > cb)
                                                        if(p[offset7] > cb)
                                                            if(p[offset8] > cb)
                                                                if(p[offset9] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset5] > cb)
                                                    if(p[offset6] > cb)
                                                        if(p[offset7] > cb)
                                                            if(p[offset8] > cb)
                                                                if(p[offset9] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset1] > cb)
                                                if(p[offset3] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(p[offset11] < c_b)
                                if(p[offset7] < c_b)
                                    if(p[offset8] < c_b)
                                        if(p[offset9] < c_b)
                                            if(p[offset10] < c_b)
                                                if(p[offset6] < c_b)
                                                    if(p[offset5] < c_b)
                                                        if(p[offset3] < c_b)
                                                        {}
                                                        else
                                                            if(p[offset12] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset12] < c_b)
                                                            if(p[offset13] < c_b)
                                                                if(p[offset14] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset12] < c_b)
                                                        if(p[offset13] < c_b)
                                                            if(p[offset14] < c_b)
                                                                if(p[offset15] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            if(p[offset11] > cb)
                                if(p[offset12] > cb)
                                    if(p[offset13] > cb)
                                        if(p[offset10] > cb)
                                            if(p[offset14] > cb)
                                                if(p[offset15] > cb)
                                                    if(p[offset1] > cb)
                                                    {}
                                                    else
                                                        if(p[offset8] > cb)
                                                            if(p[offset9] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset6] > cb)
                                                        if(p[offset7] > cb)
                                                            if(p[offset8] > cb)
                                                                if(p[offset9] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset5] > cb)
                                                    if(p[offset6] > cb)
                                                        if(p[offset7] > cb)
                                                            if(p[offset8] > cb)
                                                                if(p[offset9] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset1] > cb)
                                                if(p[offset3] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(p[offset11] < c_b)
                                if(p[offset7] < c_b)
                                    if(p[offset8] < c_b)
                                        if(p[offset9] < c_b)
                                            if(p[offset10] < c_b)
                                                if(p[offset12] < c_b)
                                                    if(p[offset13] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset5] < c_b)
                                                            {}
                                                            else
                                                                if(p[offset14] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                        else
                                                            if(p[offset14] < c_b)
                                                                if(p[offset15] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                    else if(p[offset2] < c_b)
                        if(p[offset9] > cb)
                            if(p[offset10] > cb)
                                if(p[offset11] > cb)
                                    if(p[offset8] > cb)
                                        if(p[offset12] > cb)
                                            if(p[offset13] > cb)
                                                if(p[offset14] > cb)
                                                    if(p[offset15] > cb)
                                                    {}
                                                    else
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset4] > cb)
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset3] > cb)
                                                if(p[offset4] > cb)
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset1] > cb)
                                            if(p[offset12] > cb)
                                                if(p[offset13] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(p[offset9] < c_b)
                            if(p[offset7] < c_b)
                                if(p[offset8] < c_b)
                                    if(p[offset6] < c_b)
                                        if(p[offset5] < c_b)
                                            if(p[offset4] < c_b)
                                                if(p[offset3] < c_b)
                                                    if(p[offset1] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset10] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] < c_b)
                                                        if(p[offset11] < c_b)
                                                            if(p[offset12] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset10] < c_b)
                                                    if(p[offset11] < c_b)
                                                        if(p[offset12] < c_b)
                                                            if(p[offset13] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset10] < c_b)
                                                if(p[offset11] < c_b)
                                                    if(p[offset12] < c_b)
                                                        if(p[offset13] < c_b)
                                                            if(p[offset14] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] < c_b)
                                            if(p[offset11] < c_b)
                                                if(p[offset12] < c_b)
                                                    if(p[offset13] < c_b)
                                                        if(p[offset14] < c_b)
                                                            if(p[offset15] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        if(p[offset9] > cb)
                            if(p[offset10] > cb)
                                if(p[offset11] > cb)
                                    if(p[offset8] > cb)
                                        if(p[offset12] > cb)
                                            if(p[offset13] > cb)
                                                if(p[offset14] > cb)
                                                    if(p[offset15] > cb)
                                                    {}
                                                    else
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset4] > cb)
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset3] > cb)
                                                if(p[offset4] > cb)
                                                    if(p[offset5] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset7] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset1] > cb)
                                            if(p[offset12] > cb)
                                                if(p[offset13] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(p[offset9] < c_b)
                            if(p[offset7] < c_b)
                                if(p[offset8] < c_b)
                                    if(p[offset10] < c_b)
                                        if(p[offset11] < c_b)
                                            if(p[offset6] < c_b)
                                                if(p[offset5] < c_b)
                                                    if(p[offset4] < c_b)
                                                        if(p[offset3] < c_b)
                                                        {}
                                                        else
                                                            if(p[offset12] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset12] < c_b)
                                                            if(p[offset13] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset12] < c_b)
                                                        if(p[offset13] < c_b)
                                                            if(p[offset14] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset12] < c_b)
                                                    if(p[offset13] < c_b)
                                                        if(p[offset14] < c_b)
                                                            if(p[offset15] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                else if(p[offset0] < c_b)
                    if(p[offset2] > cb)
                        if(p[offset9] > cb)
                            if(p[offset7] > cb)
                                if(p[offset8] > cb)
                                    if(p[offset6] > cb)
                                        if(p[offset5] > cb)
                                            if(p[offset4] > cb)
                                                if(p[offset3] > cb)
                                                    if(p[offset1] > cb)
                                                    {}
                                                    else
                                                        if(p[offset10] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] > cb)
                                                        if(p[offset11] > cb)
                                                            if(p[offset12] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset10] > cb)
                                                    if(p[offset11] > cb)
                                                        if(p[offset12] > cb)
                                                            if(p[offset13] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset10] > cb)
                                                if(p[offset11] > cb)
                                                    if(p[offset12] > cb)
                                                        if(p[offset13] > cb)
                                                            if(p[offset14] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] > cb)
                                            if(p[offset11] > cb)
                                                if(p[offset12] > cb)
                                                    if(p[offset13] > cb)
                                                        if(p[offset14] > cb)
                                                            if(p[offset15] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(p[offset9] < c_b)
                            if(p[offset10] < c_b)
                                if(p[offset11] < c_b)
                                    if(p[offset8] < c_b)
                                        if(p[offset12] < c_b)
                                            if(p[offset13] < c_b)
                                                if(p[offset14] < c_b)
                                                    if(p[offset15] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset4] < c_b)
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset3] < c_b)
                                                if(p[offset4] < c_b)
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset1] < c_b)
                                            if(p[offset12] < c_b)
                                                if(p[offset13] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if(p[offset2] < c_b)
                        if(p[offset4] > cb)
                            if(p[offset11] > cb)
                                if(p[offset7] > cb)
                                    if(p[offset8] > cb)
                                        if(p[offset9] > cb)
                                            if(p[offset10] > cb)
                                                if(p[offset6] > cb)
                                                    if(p[offset5] > cb)
                                                        if(p[offset3] > cb)
                                                        {}
                                                        else
                                                            if(p[offset12] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset12] > cb)
                                                            if(p[offset13] > cb)
                                                                if(p[offset14] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset12] > cb)
                                                        if(p[offset13] > cb)
                                                            if(p[offset14] > cb)
                                                                if(p[offset15] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(p[offset11] < c_b)
                                if(p[offset12] < c_b)
                                    if(p[offset13] < c_b)
                                        if(p[offset10] < c_b)
                                            if(p[offset14] < c_b)
                                                if(p[offset15] < c_b)
                                                    if(p[offset1] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset8] < c_b)
                                                            if(p[offset9] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset6] < c_b)
                                                        if(p[offset7] < c_b)
                                                            if(p[offset8] < c_b)
                                                                if(p[offset9] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset5] < c_b)
                                                    if(p[offset6] < c_b)
                                                        if(p[offset7] < c_b)
                                                            if(p[offset8] < c_b)
                                                                if(p[offset9] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset1] < c_b)
                                                if(p[offset3] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(p[offset4] < c_b)
                            if(p[offset5] > cb)
                                if(p[offset12] > cb)
                                    if(p[offset7] > cb)
                                        if(p[offset8] > cb)
                                            if(p[offset9] > cb)
                                                if(p[offset10] > cb)
                                                    if(p[offset11] > cb)
                                                        if(p[offset13] > cb)
                                                            if(p[offset6] > cb)
                                                            {}
                                                            else
                                                                if(p[offset14] > cb)
                                                                    if(p[offset15] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(p[offset12] < c_b)
                                    if(p[offset13] < c_b)
                                        if(p[offset14] < c_b)
                                            if(p[offset15] < c_b)
                                                if(p[offset1] < c_b)
                                                    if(p[offset3] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset10] < c_b)
                                                            if(p[offset11] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset8] < c_b)
                                                        if(p[offset9] < c_b)
                                                            if(p[offset10] < c_b)
                                                                if(p[offset11] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset6] < c_b)
                                                    if(p[offset7] < c_b)
                                                        if(p[offset8] < c_b)
                                                            if(p[offset9] < c_b)
                                                                if(p[offset10] < c_b)
                                                                    if(p[offset11] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(p[offset5] < c_b)
                                if(p[offset7] > cb)
                                    if(p[offset14] > cb)
                                        if(p[offset8] > cb)
                                            if(p[offset9] > cb)
                                                if(p[offset10] > cb)
                                                    if(p[offset11] > cb)
                                                        if(p[offset12] > cb)
                                                            if(p[offset13] > cb)
                                                                if(p[offset6] > cb)
                                                                {}
                                                                else
                                                                    if(p[offset15] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if(p[offset14] < c_b)
                                        if(p[offset15] < c_b)
                                            if(p[offset1] < c_b)
                                                if(p[offset3] < c_b)
                                                    if(p[offset6] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset13] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] < c_b)
                                                        if(p[offset11] < c_b)
                                                            if(p[offset12] < c_b)
                                                                if(p[offset13] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset8] < c_b)
                                                    if(p[offset9] < c_b)
                                                        if(p[offset10] < c_b)
                                                            if(p[offset11] < c_b)
                                                                if(p[offset12] < c_b)
                                                                    if(p[offset13] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(p[offset7] < c_b)
                                    if(p[offset3] < c_b)
                                        if(p[offset1] < c_b)
                                            if(p[offset6] < c_b)
                                                if(p[offset8] < c_b)
                                                {}
                                                else
                                                    if(p[offset15] < c_b)
                                                    {}
                                                    else
                                                        continue;
                                            else
                                                if(p[offset13] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset8] < c_b)
                                                if(p[offset9] < c_b)
                                                    if(p[offset10] < c_b)
                                                        if(p[offset6] < c_b)
                                                        {}
                                                        else
                                                            if(p[offset11] < c_b)
                                                                if(p[offset12] < c_b)
                                                                    if(p[offset13] < c_b)
                                                                        if(p[offset14] < c_b)
                                                                            if(p[offset15] < c_b)
                                                                            {}
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] < c_b)
                                            if(p[offset11] < c_b)
                                                if(p[offset12] < c_b)
                                                    if(p[offset8] < c_b)
                                                        if(p[offset9] < c_b)
                                                            if(p[offset6] < c_b)
                                                            {}
                                                            else
                                                                if(p[offset13] < c_b)
                                                                    if(p[offset14] < c_b)
                                                                        if(p[offset15] < c_b)
                                                                        {}
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                        else
                                                            if(p[offset1] < c_b)
                                                                if(p[offset13] < c_b)
                                                                    if(p[offset14] < c_b)
                                                                        if(p[offset15] < c_b)
                                                                        {}
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset1] < c_b)
                                                            if(p[offset13] < c_b)
                                                                if(p[offset14] < c_b)
                                                                    if(p[offset15] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(p[offset14] < c_b)
                                        if(p[offset15] < c_b)
                                            if(p[offset1] < c_b)
                                                if(p[offset3] < c_b)
                                                    if(p[offset6] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset13] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] < c_b)
                                                        if(p[offset11] < c_b)
                                                            if(p[offset12] < c_b)
                                                                if(p[offset13] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset8] < c_b)
                                                    if(p[offset9] < c_b)
                                                        if(p[offset10] < c_b)
                                                            if(p[offset11] < c_b)
                                                                if(p[offset12] < c_b)
                                                                    if(p[offset13] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                if(p[offset12] > cb)
                                    if(p[offset7] > cb)
                                        if(p[offset8] > cb)
                                            if(p[offset9] > cb)
                                                if(p[offset10] > cb)
                                                    if(p[offset11] > cb)
                                                        if(p[offset13] > cb)
                                                            if(p[offset14] > cb)
                                                                if(p[offset6] > cb)
                                                                {}
                                                                else
                                                                    if(p[offset15] > cb)
                                                                    {}
                                                                    else
                                                                        continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if(p[offset12] < c_b)
                                    if(p[offset13] < c_b)
                                        if(p[offset14] < c_b)
                                            if(p[offset15] < c_b)
                                                if(p[offset1] < c_b)
                                                    if(p[offset3] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset10] < c_b)
                                                            if(p[offset11] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset8] < c_b)
                                                        if(p[offset9] < c_b)
                                                            if(p[offset10] < c_b)
                                                                if(p[offset11] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset6] < c_b)
                                                    if(p[offset7] < c_b)
                                                        if(p[offset8] < c_b)
                                                            if(p[offset9] < c_b)
                                                                if(p[offset10] < c_b)
                                                                    if(p[offset11] < c_b)
                                                                    {}
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                        else
                            if(p[offset11] > cb)
                                if(p[offset7] > cb)
                                    if(p[offset8] > cb)
                                        if(p[offset9] > cb)
                                            if(p[offset10] > cb)
                                                if(p[offset12] > cb)
                                                    if(p[offset13] > cb)
                                                        if(p[offset6] > cb)
                                                            if(p[offset5] > cb)
                                                            {}
                                                            else
                                                                if(p[offset14] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                        else
                                                            if(p[offset14] > cb)
                                                                if(p[offset15] > cb)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if(p[offset11] < c_b)
                                if(p[offset12] < c_b)
                                    if(p[offset13] < c_b)
                                        if(p[offset10] < c_b)
                                            if(p[offset14] < c_b)
                                                if(p[offset15] < c_b)
                                                    if(p[offset1] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset8] < c_b)
                                                            if(p[offset9] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset6] < c_b)
                                                        if(p[offset7] < c_b)
                                                            if(p[offset8] < c_b)
                                                                if(p[offset9] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset5] < c_b)
                                                    if(p[offset6] < c_b)
                                                        if(p[offset7] < c_b)
                                                            if(p[offset8] < c_b)
                                                                if(p[offset9] < c_b)
                                                                {}
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset1] < c_b)
                                                if(p[offset3] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                    else
                        if(p[offset9] > cb)
                            if(p[offset7] > cb)
                                if(p[offset8] > cb)
                                    if(p[offset10] > cb)
                                        if(p[offset11] > cb)
                                            if(p[offset6] > cb)
                                                if(p[offset5] > cb)
                                                    if(p[offset4] > cb)
                                                        if(p[offset3] > cb)
                                                        {}
                                                        else
                                                            if(p[offset12] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                    else
                                                        if(p[offset12] > cb)
                                                            if(p[offset13] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset12] > cb)
                                                        if(p[offset13] > cb)
                                                            if(p[offset14] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset12] > cb)
                                                    if(p[offset13] > cb)
                                                        if(p[offset14] > cb)
                                                            if(p[offset15] > cb)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if(p[offset9] < c_b)
                            if(p[offset10] < c_b)
                                if(p[offset11] < c_b)
                                    if(p[offset8] < c_b)
                                        if(p[offset12] < c_b)
                                            if(p[offset13] < c_b)
                                                if(p[offset14] < c_b)
                                                    if(p[offset15] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset4] < c_b)
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset3] < c_b)
                                                if(p[offset4] < c_b)
                                                    if(p[offset5] < c_b)
                                                        if(p[offset6] < c_b)
                                                            if(p[offset7] < c_b)
                                                            {}
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset1] < c_b)
                                            if(p[offset12] < c_b)
                                                if(p[offset13] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                else
                    if(p[offset7] > cb)
                        if(p[offset8] > cb)
                            if(p[offset9] > cb)
                                if(p[offset6] > cb)
                                    if(p[offset5] > cb)
                                        if(p[offset4] > cb)
                                            if(p[offset3] > cb)
                                                if(p[offset2] > cb)
                                                    if(p[offset1] > cb)
                                                    {}
                                                    else
                                                        if(p[offset10] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] > cb)
                                                        if(p[offset11] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset10] > cb)
                                                    if(p[offset11] > cb)
                                                        if(p[offset12] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset10] > cb)
                                                if(p[offset11] > cb)
                                                    if(p[offset12] > cb)
                                                        if(p[offset13] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] > cb)
                                            if(p[offset11] > cb)
                                                if(p[offset12] > cb)
                                                    if(p[offset13] > cb)
                                                        if(p[offset14] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(p[offset10] > cb)
                                        if(p[offset11] > cb)
                                            if(p[offset12] > cb)
                                                if(p[offset13] > cb)
                                                    if(p[offset14] > cb)
                                                        if(p[offset15] > cb)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                        else
                            continue;
                    else if(p[offset7] < c_b)
                        if(p[offset8] < c_b)
                            if(p[offset9] < c_b)
                                if(p[offset6] < c_b)
                                    if(p[offset5] < c_b)
                                        if(p[offset4] < c_b)
                                            if(p[offset3] < c_b)
                                                if(p[offset2] < c_b)
                                                    if(p[offset1] < c_b)
                                                    {}
                                                    else
                                                        if(p[offset10] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                else
                                                    if(p[offset10] < c_b)
                                                        if(p[offset11] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                            else
                                                if(p[offset10] < c_b)
                                                    if(p[offset11] < c_b)
                                                        if(p[offset12] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                        else
                                            if(p[offset10] < c_b)
                                                if(p[offset11] < c_b)
                                                    if(p[offset12] < c_b)
                                                        if(p[offset13] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                    else
                                        if(p[offset10] < c_b)
                                            if(p[offset11] < c_b)
                                                if(p[offset12] < c_b)
                                                    if(p[offset13] < c_b)
                                                        if(p[offset14] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                else
                                    if(p[offset10] < c_b)
                                        if(p[offset11] < c_b)
                                            if(p[offset12] < c_b)
                                                if(p[offset13] < c_b)
                                                    if(p[offset14] < c_b)
                                                        if(p[offset15] < c_b)
                                                        {}
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
            }


            if(total >= MAX_CORNERS)
            {
                goto mv_fast_end;
            }

            pXyCorner[total].x = (float)x;
            pXyCorner[total].y = (float)y;
            pXyCorner[total].bPreTrack = 0;
            total++;
        }
         y = y + stepy;
    }

mv_fast_end:

    *num_corners = total;

}

/************************************************************************/
/* 四舍五入取整
*/
/************************************************************************/
int mround(float f_digit)
{
    if(f_digit>0.1)
        return (int)(f_digit + 0.5);
    else
        return (int)(f_digit - 0.5);
}

int getSubMatrix(unsigned char **  imagePyr1, unsigned char **  imagePyr2,int L_it,int px,int py,int tx,int ty,short * diff_I)
{

#if NEON == 1
    unsigned char * image1 = *(imagePyr1 + L_it);
    unsigned char * image2 = *(imagePyr2 + L_it);
    int width_L = g_pLkInner->SizeOfPyr[L_it].width;
    int height_L = g_pLkInner->SizeOfPyr[L_it].height;
    int i = 0;
    int j = 0;
    int neonCount = wx - 8;
    //求得偏导
    uint8x8_t src1;
    uint8x8_t src2;
    int16x8_t dst;

    if (py - wy < 0 || py + wy > height_L-1 ||  ty - wy < 0 ||
        ty + wy > height_L-1 || px - wx < 0 || px + wx > width_L
        || tx - wx < 0 || tx + wx > width_L )
    {
        return 0;
    }

    for(j = -wy; j <= wy; j++)
    {
        unsigned char* in1 = image1 + (j + py) * width_L + px;
        unsigned char* in2 = image2 + (j + ty) * width_L + tx;
        uint16x8_t src3, src4;
        int16x8_t src5, src6;

        for(i = -wx; i < neonCount; i += 8)
        {
            src1 = vld1_u8(in1 + i);
            src2 = vld1_u8(in2 + i);

            src3 = vmovl_u8(src1);
            src4 = vmovl_u8(src2);

            src5 = vreinterpretq_s16_u16(src3);
            src6 = vreinterpretq_s16_u16(src4);

            dst = vsubq_s16(src5,  src6);
            vst1q_s16(diff_I, dst);
            diff_I += 8;
        }

        for(; i <= wx; i++)
        {
           *(diff_I++) = *(in1 + i) - *(in2 + i);
        }
    }

    return 1;

#else

    unsigned char * image1 = *(imagePyr1 + L_it);
    unsigned char * image2 = *(imagePyr2 + L_it);
    int width_L = g_pLkInner->SizeOfPyr[L_it].width;
    int height_L = g_pLkInner->SizeOfPyr[L_it].height;
    int i, j;
    //求得偏导

    if (py - wy < 0 || py + wy > height_L-1 ||  ty - wy < 0 ||
        ty + wy > height_L-1 || px - wx < 0 || px + wx > width_L
        || tx - wx < 0 || tx + wx > width_L )
    {
        return 0;
    }

    for(j = -wy; j <= wy; j++)
    {
        unsigned char* in1 = image1 + (j + py) * width_L + px;
        unsigned char* in2 = image2 + (j + ty) * width_L + tx;
        for(i = -wx; i <= wx; i++)
        {
            *(diff_I) = *(in1 + i) - *(in2 + i);
            diff_I++;
        }
    }
    return 1;

#endif
}

void getbk(int * b1,int *  b2, short * src0, short * srcx, short * srcy)
{

#if NEON == 1
    int i = 0;
    int32x4_t resultA = vdupq_n_s32(0);
    int32x4_t resultB = vdupq_n_s32(0);
    int16x4_t tempSrc0 = vdup_n_s16(0);
    int16x4_t tempSrcx = vdup_n_s16(0);
    int16x4_t tempSrcy = vdup_n_s16(0);
    int32x2_t vec64LowA;
    int32x2_t vec64HighA;
    int32x2_t vec64a;
    int32x2_t vec64LowB;
    int32x2_t vec64HighB;
    int32x2_t vec64b;
    int32x2_t result;

    for(i = 0; i < OPTICAL_WINDOW_COUNT; i += 4)
    {
        tempSrc0 = vld1_s16(src0 + i);
        tempSrcx = vld1_s16(srcx + i);
        resultA = vmlal_s16(resultA, tempSrc0, tempSrcx);
        tempSrcy = vld1_s16(srcy + i);
        resultB = vmlal_s16(resultB, tempSrc0, tempSrcy);
    }

    vec64LowA = vget_low_s32(resultA);
    vec64HighA = vget_high_s32(resultA);
    vec64a = vadd_s32(vec64LowA, vec64HighA);

    vec64LowB = vget_low_s32(resultB);
    vec64HighB = vget_high_s32(resultB);
    vec64b = vadd_s32(vec64LowB, vec64HighB);

    result = vpadd_s32(vec64a, vec64b);
    result = vshr_n_s32(result, 1);

    *b1 = vget_lane_s32(result, 0);
    *b2 = vget_lane_s32(result, 1);
#elif NEON_ASM == 1
    //int count = OPTICAL_WINDOW_COUNT;
    __asm volatile (
                "VMOV.I32 q0,#0x00\t\n"
                "VMOV.I32 q1,#0x00\t\n"
                "MOV      r6,#0x7C\t\n"
                "LOOP: \t\n"
                "VLD1.16   {d4},[%[src0]]!\t\n"
                "VLD1.16   {d5},[%[srcx]]!\t\n"
                "VLD1.16   {d6},[%[srcy]]!\t\n"
                "VMLAL.S16 q0,d4,d5\t\n"
                "VMLAL.S16 q1,d4,d6\t\n"
                "SUBS      r6,r6,#4\t\n"
                "BGT       LOOP\t\n"
                "VADD.I32  d0,d0,d1\t\n"
                "VADD.I32  d2,d2,d3\t\n"
                "VPADD.I32 d7,d0,d2\t\n"
                "VSHR.S32  d7,d7,#1\t\n"
                "VST1.32   d7[0],[%[b1]]!\t\n"
                "VST1.32   d7[1],[%[b2]]!\t\n"
                : [b1]"+r"(b1), [b2]"+r"(b2)
                : [src0]"r"(src0), [srcx]"r"(srcx), [srcy]"r"(srcy)
                : "memory", "r6", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"
                );
#else

    int i, cyc;

    *b1 = 0;
    *b2 = 0;
    cyc = (2 * wx + 1) * (2 * wy + 1);
    for(i = 0;i < cyc ; i++)
    {
        *b1 += src0[i] * srcx[i];
        *b2 += src0[i] * srcy[i];
    }

    *b1 = *b1 >> 1;// 左移右移都是有意义的 请勿随便改动
    *b2 = *b2 >> 1;// 左移右移都是有意义的 请勿随便改动

#endif
}

void getIx_Iy(unsigned char ** imagePyr1,int L_it,int px,int py, short * Ix, short * Iy)
{
#if NEON == 1
    const unsigned char *image1 = *(imagePyr1+L_it);
    int width_L = g_pLkInner->SizeOfPyr[L_it].width;
    int i, j;
    int ySatrt = py - wy;
    int xStart = px - wx;
    int yCount = py + wy;
    int xCount = px + wx;
    int neonCount = xCount - 8;
    //求得偏导
    int16x8_t IxResult;
    int16x8_t IyResult;
    uint8x8_t Ixdata1;
    uint8x8_t Ixdata2;
    uint8x8_t Iydata1;
    uint8x8_t Iydata2;

    for(j = ySatrt;j <= yCount; j++)
    {
        const unsigned char* in1 = image1 + j * width_L;
        const unsigned char* in3 = image1 + (j + 1) * width_L;
        const unsigned char* in4 = image1 + (j - 1) * width_L;
        uint16x8_t tempSrc1, tempSrc2, tempSrc3, tempSrc4;
        int16x8_t tempX1, tempX2, tempY1, tempY2;
        for(i = xStart; i <= neonCount; i += 8)
        {
            Ixdata1 = vld1_u8(in1 + i + 1);
            Ixdata2 = vld1_u8(in1 + i - 1);
            tempSrc1 = vmovl_u8(Ixdata1);
            tempSrc2 = vmovl_u8(Ixdata2);
            tempX1 = vreinterpretq_s16_u16(tempSrc1);
            tempX2 = vreinterpretq_s16_u16(tempSrc2);
            IxResult = vsubq_s16(tempX1,  tempX2);

            Iydata1 = vld1_u8(in3 + i);
            Iydata2 = vld1_u8(in4 + i);
            tempSrc3 = vmovl_u8(Iydata1);
            tempSrc4 = vmovl_u8(Iydata2);
            tempY1 = vreinterpretq_s16_u16(tempSrc3);
            tempY2 = vreinterpretq_s16_u16(tempSrc4);
            IyResult = vsubq_s16(tempY1,  tempY2);

            vst1q_s16(Ix, IxResult);
            vst1q_s16(Iy, IyResult);
            Ix += 8;
            Iy += 8;
        }
        for(; i <= xCount; i++)
        {
           *(Ix++) = *(in1 + i + 1) - *(in1 + i - 1);
           *(Iy++) = *(in3 + i) - *(in4 + i);
        }
    }

#else
    const unsigned char *image1 = *(imagePyr1+L_it);
    int width_L = g_pLkInner->SizeOfPyr[L_it].width;
    int i,j;
    int ySatrt = py - wy;
    int xStart = px - wx;
    int yCount = py + wy;
    int xCount = px + wx;
    //求得偏导
    int offset_I = 0;
    for(j = ySatrt;j <= yCount; j++)
    {
        const unsigned char* in1 = image1 + j * width_L;
        const unsigned char* in3 = image1 + (j + 1) * width_L;
        const unsigned char* in4 = image1 + (j - 1) * width_L;
        for(i = xStart; i <= xCount; i++)
        {
            *(Ix + offset_I) = *(in1 + i + 1) - *(in1 + i - 1);     // 少除以2 在以后补上
            *(Iy + offset_I) = *(in3 + i) - *(in4 + i); //少除以2 在以后补上
            offset_I++;
        }
    }
#endif
}


unsigned short getInvG(float * niG, unsigned char ** imagePyr,int L_it,int px,int py,\
               short * Ix, short * Iy)
{
#if NEON == 1
    int i = 0;
    float gA = 0, gB = 0, gC = 0;
    float valueG = 0;
    float f = 0;

    int16x4_t neonIx = vdup_n_s16(0);
    int16x4_t neonIy = vdup_n_s16(0);
    int32x4_t resultGA = vdupq_n_s32(0);
    int32x4_t resultGB = vdupq_n_s32(0);
    int32x4_t resultGC = vdupq_n_s32(0);
    int32x2_t tempGA;
    int32x2_t tempGB;
    int32x2_t tempGC;
    int32x2_t tempCA;
    int32x2_t tempBC;
    int32x4_t tempResult;
    float32x4_t gShift = vdupq_n_f32(0.25f);
    float32x4_t resultG = vdupq_n_f32(0.0f);
    float32x4_t resultF;
    float32x4_t resultNiG;

    //得到偏导数
    getIx_Iy(imagePyr, L_it, px, py, Ix, Iy);

    for(i = 0; i < OPTICAL_WINDOW_COUNT; i += 4)
    {
       neonIx = vld1_s16(Ix + i);
       neonIy = vld1_s16(Iy + i);
       resultGC = vmlal_s16(resultGC, neonIx, neonIy);
       resultGA = vmlal_s16(resultGA, neonIx, neonIx);
       resultGB = vmlal_s16(resultGB, neonIy, neonIy);
    }
    tempGA = vadd_s32(vget_low_s32(resultGA), vget_high_s32(resultGA));
    tempGC = vadd_s32(vget_low_s32(resultGC), vget_high_s32(resultGC));
    tempCA = vpadd_s32(tempGC, tempGA);
    tempGB = vadd_s32(vget_low_s32(resultGB), vget_high_s32(resultGB));
    tempBC = vpadd_s32(tempGB, tempGC);

    tempResult = vcombine_s32(tempBC, tempCA);
    resultG = vcvtq_f32_s32(tempResult);
    resultG = vmulq_f32(resultG, gShift);
    gA = vgetq_lane_f32(resultG, 3);
    gB = vgetq_lane_f32(resultG, 0);
    gC = vgetq_lane_f32(resultG, 1);

    valueG = (gA * gB - gC * gC);// 左移右移都是有意义的 请勿随便改动 ||G||
    //若行列式为0则无逆
    // 0 = -16到16
    if(valueG < 0.001 && valueG > -0.001)
    {
        return 0;
    }

    f = 1.0f / valueG;
    resultF = vdupq_n_f32(f);
    resultF = vsetq_lane_f32(-f, resultF, 1);
    resultF = vsetq_lane_f32(-f, resultF, 2);
    resultNiG = vmulq_f32(resultG, resultF);
    vst1q_f32(niG, resultNiG);
    return 1;

#else

    int i;
    int tempA, tempB, tempC;
    float gA, gB, gC;
    float valueG;
    float f;


    //得到偏导数
    getIx_Iy(imagePyr, L_it, px, py, Ix, Iy);

    tempA = 0;
    tempB = 0;
    tempC = 0;

    // 求叠加
    for(i=0;i<(2*wx+1)*(2*wy+1);i++)
    {
        tempA+=*(Ix+i)*(*(Ix+i));//Ix的平方项
        tempB+=*(Iy+i)*(*(Iy+i));//Iy的平方项
        tempC+=*(Ix+i)*(*(Iy+i));//Ix * Iy
    }

    gB = tempB / 4.0f;
    gA = tempA / 4.0f;
    gC = tempC / 4.0f;
    valueG = (gA * gB - gC * gC);// 左移右移都是有意义的 请勿随便改动 ||G||

    //若行列式为0则无逆
    // 0 = -16到16
    if(valueG<0.001 && valueG>-0.001)
    {
        return 0;
    }

    f = 1.0f / valueG;
    //printf("Rsmall:%d\n",_lmbd(0x00000001,(unsigned int)valueG));
    //求逆
    niG[0] = f*gB;   // 左移右移都是有意义的 请勿随便改动
    niG[1] =- f*gC;
    niG[3] = f*gA;
    niG[2] = niG[1];

    return 1;

 #endif
}


 unsigned short mvTrajecyOpticalFlowPyrLK(CornerPoint ** corner, CornerPoint * cornernext,\
                                     int * numOfcorner, unsigned short * found_found)
{

    CornerPoint gl,vl,hh;
    //CPOINU  spe_gl,spe_vl,spe_hh;
    int i, j, k;
    int diedai = 4 ; // 实际迭代次数为再加1
    int py, px;//迭代次数

    unsigned short found;//找到对应点

    //申请空间
    short Ix_pre[OPTICAL_WINDOW_COUNT];
    short Iy_pre[OPTICAL_WINDOW_COUNT];
    short diff_I[OPTICAL_WINDOW_COUNT];

    float sif;

    //存逆矩阵
    float niG[4] = {0.0,0.0,0.0,0.0};
    float f_x, f_y;
    int nState;
    int nNumCorner = *numOfcorner;

    int bk1, bk2;

    memset(Ix_pre, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);
    memset(Iy_pre, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);
    memset(diff_I, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);

    for(i = 0; i < nNumCorner; i++)
    {
        //每一个强角点初始化gl dl

        if(!found_found[i])
        {
            continue;
        }

        gl.x = 0.0;
        gl.y = 0.0;

        // dl.x=0.0;
        // dl.y=0.0;
        found = 1;
        f_x = corner[i]->x;
        f_y = corner[i]->y;

        for(j = XUNHUAN_K_IT; j >= 0; j--)
        {
            sif = 1.0f / (1 << j);

            py = mround(f_y * sif); // round  比int 有效
            // printf("%d,",py);
            px = mround(f_x * sif);

            found = (py) > 5 && py < g_pLkInner->SizeOfPyr[j].height - 2 \
                && (px) > 5 && px < g_pLkInner->SizeOfPyr[j].width - 2\
                && getInvG(niG, g_pLkInner->prePyr, j, px, py, Ix_pre, Iy_pre)
                /*&& gl.x<30 \
                && gl.y<30*/ ;
            vl.x = 0.0;
            vl.y = 0.0;

            k  = found ? diedai : -1;
            //move_used_x =  mround(gl.x);
            //move_used_y =  mround(gl.y);
            for(;k>=0;k--)
            {

                //将两者合并2012 12 21 修改
                nState = getSubMatrix(g_pLkInner->prePyr, g_pLkInner->thisPyr,j,px,py,\
                    px + mround(vl.x + gl.x), py+mround(vl.y + gl.y), diff_I);
                if (!nState)
                {
                    found = 0;
                    break;
                }

                getbk(&bk1, &bk2, diff_I, Ix_pre, Iy_pre);

                hh.x = niG[0] * bk1 + niG[1] * bk2;
                hh.y = niG[2] * bk1 + niG[3] * bk2;
                //printf("vl:%9.5f,bk1:%9d,HH:x=%.5f\n",niG[0],bk1, mmpyFansI(niG[0],bk1));
                vl.x += hh.x;
                vl.y += hh.y;

                found = vl.x < 25 && vl.y < 25 && vl.x>-25 && vl.y>-25  ;//每层的跨度小于23 即5
                k = (int)((found && fabs(hh.x) > 0.01 && fabs(hh.y) > 0.01) * k );
            }
            j = j * found;

            gl.x=(vl.x + gl.x) * 2;
            gl.y=(vl.y + gl.y) * 2;


        }

        found_found[i] = found;

        ((cornernext+i))->x = corner[i]->x + found *  gl.x / 2;// gl.x/2;
        ((cornernext+i))->y = corner[i]->y + found *  gl.y / 2;

    }
    return 1;
}

 unsigned short mvCalcOpticalFlowPyrLK(CornerPoint * corner, CornerPoint * cornernext,\
                                     int * numOfcorner, unsigned short * found_found)
{
    CornerPoint gl,vl,hh;
    //CPOINU  spe_gl,spe_vl,spe_hh;
    int i,j,k;
    int diedai = 4 ; // 实际迭代次数为再加1
    int  py,px;//迭代次数

    unsigned short found;//找到对应点
    //申请空间
    short Ix_pre[OPTICAL_WINDOW_COUNT];
    short Iy_pre[OPTICAL_WINDOW_COUNT];
    short diff_I[OPTICAL_WINDOW_COUNT];

    float sif;

    //存逆矩阵
    float niG[4]={0.0, 0.0, 0.0, 0.0};
    float f_x,f_y;
    int nState;

    int bk1, bk2;

    memset(Ix_pre, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);
    memset(Iy_pre, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);
    memset(diff_I, 0, sizeof(short) * OPTICAL_WINDOW_COUNT);

    //t1 = Roseek_GetTickCount();
    for(i = 0;i <*numOfcorner;i++)
    {
        //每一个强角点初始化gl dl
        gl.x=0.0;
        gl.y=0.0;

        //dl.x=0.0;
        //dl.y=0.0;
        found = 1;
        f_x = (*(corner+i)).x;
        f_y = (*(corner+i)).y;

        if (!found_found[i])
        {
            continue;
        }

        for(j = XUNHUAN_K_IT;j >= 0; j--)
        {
            sif = 1.0f / (1 << j);
            py = mround(f_y * sif); // round比int 有效
            // printf("%d,",py);
            px = mround(f_x * sif);

            found = (py) >5 && py < g_pLkInner->SizeOfPyr[j].height - 2 \
                && (px) > 5 && px < g_pLkInner->SizeOfPyr[j].width - 2\
                && getInvG(niG,g_pLkInner->prePyr,j,px,py,Ix_pre,Iy_pre)
                /*&& gl.x<30 \
                && gl.y<30*/ ;
            vl.x = 0.0;
            vl.y = 0.0;

            k  =found ? diedai : -1;
            // move_used_x =  mround(gl.x);
            //move_used_y =  mround(gl.y);
            for(;k>=0;k--)
            {
                // 将两者合并 2012 12 21 修改
                nState = getSubMatrix(g_pLkInner->prePyr, g_pLkInner->thisPyr, j, px, py,\
                    px+mround(vl.x+gl.x), py+mround(vl.y+gl.y), diff_I);
                if (!nState)
                {
                    found = 0;
                    break;
                }

                getbk( &bk1, &bk2, diff_I, Ix_pre, Iy_pre);

                hh.x = niG[0] * bk1 + niG[1] * bk2;
                hh.y = niG[2] * bk1 + niG[3] * bk2;
                //printf("vl:%9.5f,bk1:%9d,HH:x=%.5f\n",niG[0],bk1, mmpyFansI(niG[0],bk1));
                vl.x += hh.x;
                vl.y += hh.y;

                found = vl.x<25 && vl.y<25 && vl.x>-25 && vl.y>-25  ;//每层的跨度小于23 即5
                k = (int)( (found && fabs(hh.x)>0.01 && fabs(hh.y)>0.01 ) * k );
            }

            j = j * found;

            gl.x=(vl.x + gl.x) * 2;
            gl.y=(vl.y + gl.y) * 2;

        }

        found_found[i] = found;

        (cornernext+i)->x = (corner+i)->x + found *  gl.x / 2;// gl.x/2;
        (cornernext+i)->y = (corner+i)->y + found *  gl.y / 2;//

    }
    return 1;
}

void mvAddTrajecy()
{
    int  i,j;
    int nNewCornerNum = g_pLkInner->corner_num ;
    int nTrajecNum = g_pLkInner->nTrajecyNum ;
    CornerPoint *pNewCorner = NULL;
    CornerPoint *pTrackConer = NULL;
    Trajectory *pTrajecy = NULL;
    unsigned char bNearCorner;
    float  fDisTresh = 8;

    //将检测到的点加入到轨迹中去
    for ( i = 0 ;  i < nNewCornerNum; i++ )
    {
        pNewCorner = g_pLkInner->cornersA + i;
        bNearCorner = 0;

        if (g_pLkInner->nTrajecyNum >= MAX_TRAJECY_NUM)
        {
            continue;
        }

        //将每一个新检测到的角点跟所有轨迹里面最后一个角点进行距离检测，小于fDisTresh则为相近角点不增加新的轨迹，反之增加新的轨迹；
        for ( j = 0; j < nTrajecNum; j++ )
        {
            pTrajecy = g_pLkInner->pTrajecySet + j;

            assert(TRAJECY_INVALID != pTrajecy->ntrackId && pTrajecy->PoitNum);

            pTrackConer = pTrajecy->point + ( (pTrajecy->PoitNum - 1)& MAX_TRAJECY_POINT_BIT);

            if (PNorm(pTrackConer, pNewCorner) < fDisTresh)//小于fDisTresh则表明是相近的角点，不属于新增角点
            {
                bNearCorner = 1;//一旦发现是邻近角点则跳出与剩余角点的距离分析
                break;
            }

        }

        if (!bNearCorner)
        {
            addTrajectory(g_pLkInner->pTrajecySet + g_pLkInner->nTrajecyNum,g_pLkInner->nTrajecyNum,pNewCorner);

            g_pLkInner->nTrajecyNum++;
        }

    }

}

void mvGetLKTrajecy(Trajectory **pLkTrajecy, int *pTrajecNum)
{
    *pLkTrajecy = g_pLkInner->pTrajecySet;
    *pTrajecNum = g_pLkInner->nTrajecyNum;
}

int OpticalAnalyze(unsigned char*image, int w, int h)
{
    int i,j;
    int nSumCorner;
    int nFB = 1;
    float fNormDis;
    float fFBErrorThresh;
    unsigned char uReplace = 0;
    unsigned char **mid_pyr = NULL;
    Trajectory  *pTrajecy = NULL;

    //初始化光流内存分配
    mvLKinit(w, h);

    //第0帧处理,将第一帧得到的轨迹都加进去
    if(!g_pLkInner->nGetPreImg)
    {
        //printf("track init succeed 1\n");

        memcpy(g_pLkInner->thisPyr[0], image, h * w * sizeof(unsigned char));

        //构建金字塔LK
        myCreatePyr(g_pLkInner->thisPyr, 0);

        //对当前图像进行角点检测
        oast9_16(g_pLkInner->thisPyr[0], w, h, 10, &nSumCorner, g_pLkInner->cornersA, 0, h, 0, w);

        //将这些新角点添加到轨迹中去
        if(nSumCorner > MAX_TRAJECY_NUM)
        {
            int nt = mround(nSumCorner * 1.0f / MAX_TRAJECY_NUM);
            int tolnum = (int)floatMin(nSumCorner *  1.0f / nt, MAX_TRAJECY_NUM);

            for (i = 0 ; i < tolnum; i++)
            {
                pTrajecy = g_pLkInner->pTrajecySet + i;
                addTrajectory(pTrajecy, i, g_pLkInner->cornersA + i * nt);
            }
            g_pLkInner->nTrajecyNum = tolnum;
        }
        else
        {
            for (i = 0 ; i < nSumCorner; i++)
            {
                pTrajecy = g_pLkInner->pTrajecySet + i;
                addTrajectory(pTrajecy, i, g_pLkInner->cornersA + i);
            }
             g_pLkInner->nTrajecyNum = nSumCorner;
        }

        MY_SWAP(g_pLkInner->prePyr, g_pLkInner->thisPyr, mid_pyr);
        g_pLkInner->nGetPreImg = 1;

        assert(g_pLkInner->nTrajecyNum <= MAX_TRAJECY_NUM);
    }
    else
    {

        memcpy(g_pLkInner->thisPyr[0], image, h * w * sizeof(unsigned char));

        //获得上一次的轨迹要跟踪的角点指针
        for (i = 0 ; i < g_pLkInner->nTrajecyNum ; i++)
        {
            pTrajecy = g_pLkInner->pTrajecySet + i;
            assert(TRAJECY_INVALID != pTrajecy->ntrackId);
            g_pLkInner->LastTrajCorn[i] = pTrajecy->point +
                    ((pTrajecy->PoitNum - 1) & MAX_TRAJECY_POINT_BIT);//获得所有轨迹上面最近的一组点（即上一次跟踪的角点）
        }

        //printf("nTrajecyNum = %d\n", g_pLkInner->nTrajecyNum);

        //对轨迹点进行LK跟踪，features_found初始化默认为1
        memcpy(g_pLkInner->features_found,\
            g_pLkInner->features_found_init, sizeof(unsigned short) * g_pLkInner->nTrajecyNum);

        //构建金字塔LK
        myCreatePyr(g_pLkInner->thisPyr, 0);//0.35ms左右

        //利用光流算法估计上一帧中检测的角点在当前帧中所在的位置，LastTrajCorn<->cornersB
        mvTrajecyOpticalFlowPyrLK(g_pLkInner->LastTrajCorn, g_pLkInner->cornersB,\
            &g_pLkInner->nTrajecyNum, g_pLkInner->features_found);

        //指针进行交换
        MY_SWAP(g_pLkInner->prePyr, g_pLkInner->thisPyr, mid_pyr);

        if (nFB) //逆向光流计算
        {
            mvCalcOpticalFlowPyrLK(g_pLkInner->cornersB, g_pLkInner->cornersC,\
                &g_pLkInner->nTrajecyNum, g_pLkInner->features_found);
        }


        //将found状态位1的进行---》更新轨迹！！
        for ( i = 0 ; i < g_pLkInner->nTrajecyNum; i++)
        {
            if (g_pLkInner->features_found[i])
            {
                if (nFB)
                {

                    fNormDis = PNorm(g_pLkInner->LastTrajCorn[i],(g_pLkInner->cornersC + i) );

                    fFBErrorThresh  = 2;

                    //A->B;B->C ,如果误差B，C误差很小,
                    if ( fNormDis < fFBErrorThresh && fNormDis > 0.1)
                    {
                        updataTrajectory(g_pLkInner->pTrajecySet + i,g_pLkInner->cornersB + i);

                    }
                    else
                    {
                        g_pLkInner->features_found[i] = 0;
                    }
                }
            }
        }

        //删除没有更新到的轨迹
        for ( i = 0 ; i < g_pLkInner->nTrajecyNum;i++)
        {
            if(!g_pLkInner->features_found[i])
            {
                uReplace = 0;

                for (j = g_pLkInner->nTrajecyNum -1 ; j >= i+ 1 ;j--)
                {
                    if (g_pLkInner->features_found[j])
                    {
                        uReplace = 1;
                        CopyTrajectory(g_pLkInner->pTrajecySet + i,g_pLkInner->pTrajecySet + j);
                        DelTrajectory(g_pLkInner->pTrajecySet + j);
                        g_pLkInner->nTrajecyNum--;
                        break;
                    }
                    else
                    {
                        DelTrajectory(g_pLkInner->pTrajecySet + j);
                        g_pLkInner->nTrajecyNum--;
                    }
                }

                if (!uReplace)
                {
                    DelTrajectory(g_pLkInner->pTrajecySet + i);
                    g_pLkInner->nTrajecyNum--;
                }
            }
        }

        //添加新的轨迹（先角点检测，将其中原理轨迹的点进行添加）
        nSumCorner = 0;

        oast9_16(g_pLkInner->prePyr[0], w, h, 10, &nSumCorner, g_pLkInner->cornersA, 0, h, 0, w);
        g_pLkInner->corner_num = nSumCorner;

        mvAddTrajecy();
    }
    return 1;
}

