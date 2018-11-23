#include "fast_corner_detection.h"
#include "helpers/utility.h"

void fast9_16(const unsigned char *image, const int imageWidth, const int imageHeight, int threshold,
             int startX, int endX, int startY, int endY, int* cornerNumber, FloatPoint *fastCorner)
{
    int total = 0;
    //int stepx = intMax((endX - startX) / 240, 1);
    //int stepy = intMax((endY - startY) / 80, 1);
    int stepx = 3;
    int stepy = 2;

    register int x, y;
    register int offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, \
        offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
    register int width;
    int cb ;
    int c_b;
    int start_y, End_y;

    start_y = startY;
    End_y = endY;

    offset0 = (-3) + (0) * imageWidth;
    offset1 = (-3) + (-1) * imageWidth;
    offset2 = (-2) + (-2) * imageWidth;
    offset3 = (-1) + (-3) * imageWidth;
    offset4 = (0) + (-3) * imageWidth;
    offset5 = (1) + (-3) * imageWidth;
    offset6 = (2) + (-2) * imageWidth;
    offset7 = (3) + (-1) * imageWidth;
    offset8 = (3) + (0) * imageWidth;
    offset9 = (3) + (1) * imageWidth;
    offset10 = (2) + (2) * imageWidth;
    offset11 = (1) + (3) * imageWidth;
    offset12 = (0) + (3) * imageWidth;
    offset13 = (-1) + (3) * imageWidth;
    offset14 = (-2) + (2) * imageWidth;
    offset15 = (-3) + (1) * imageWidth;

    width = imageWidth;

    for(y = start_y + stepy; y < End_y; y++)
    {
        x = startX;
        while(1)
        {
            x = x + stepx;

            if(x > endX)
                break;
            else
            {
                register const unsigned char* const p = image + y * width + x;

                cb = *p + threshold;
                c_b = *p - threshold;
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


            if(total >= MAX_FAST_CORNERS)
            {
                goto mv_fast_end;
            }

            fastCorner[total].x = (float)x;
            fastCorner[total].y = (float)y;
            total++;
        }
         y = y + stepy;
    }

mv_fast_end:

    *cornerNumber = total;

}
