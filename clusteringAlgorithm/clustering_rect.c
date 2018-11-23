#include "clustering_rect.h"
#include "helpers/utility.h"
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include <string.h>

/*
Function process:
+ If rect1 is in the neighbourhood of rect2, return 1
*/
static int  predicateRect(const float eps, const ResultRect r1, const ResultRect r2)
{
    int ret = 0;

    float delta = (float)(eps * (intMin(r1.width, r2.width) + intMin(r1.height, r2.height)) * 0.5);

    ret = abs(r1.x - r2.x) <= delta &&
        abs(r1.y - r2.y) <= delta &&
        abs(r1.x + r1.width - r2.x - r2.width) <= delta &&
        abs(r1.y + r1.height - r2.y - r2.height) <= delta;

    return ret;
}

/*
Function process:
+ Clustered the list of rects.
*/
static int clusteredRects(const ResultRect *srcRect, const int count, const float eps, int *treeNode, int *labelsNode)
{
    int i, j;
    int N = count;
    const ResultRect *vec = srcRect;
    const int RANK = 1;

    int *nodes = treeNode;
    int *labels = labelsNode;
    int k, root, root2, parent;
    int nclasses = 0;

    /* Build the queue element as the root node of the forest */
    for (i = 0; i < (N << 1); i++)
    {
        nodes[i++] = -1;
        nodes[i] = 0;
    }

    /* loop through all the elements */
    for (i = 0; i < N; i++)
    {
        root = i;

        /* find the root of each elements */
        while (nodes[(root << 1)] >= 0)
        {
            root = nodes[(root << 1)];
        }

        for (j = 0; j < N; j++)
        {
            /*
            * if i, j elements are not the same one, and vec[i] in the neighbourhood of vec[j], merged rects
            */
            if (i == j || !predicateRect(eps, vec[i], vec[j]))
            {
                continue;
            }

            root2 = j;

            /* find the root of this elements */
            while (nodes[(root2 << 1)] >= 0)
            {
                root2 = nodes[(root2 << 1)];
            }

            /* not in the same tree can be merged */
            if (root2 != root)
            {
                int rank = nodes[(root << 1) + RANK], rank2 = nodes[(root2 << 1) + RANK];

                /* 秩小的树归入到秩大的树中 */
                if (rank > rank2)
                {
                    nodes[(root2 << 1)] = root;
                }
                else
                {
                    nodes[(root << 1)] = root2;
                    nodes[(root2 << 1) + RANK] += (rank == rank2);/* 秩相等的时候才改变树的秩 */
                    root = root2;
                }

                k = j;
                /* compress the path from node2 to root */
                while ((parent = nodes[(k << 1)]) >= 0)
                {
                    nodes[(k << 1)] = root;
                    k = parent;
                }

                /* compress the path from node to root */
                k = i;
                while ((parent = nodes[(k << 1)]) >= 0)
                {
                    nodes[(k << 1)] = root;
                    k = parent;
                }
            }
        }
    }

    /* Final O(N) pass: enumerate classes */
    for (i = 0; i < N; i++)
    {
        root = i;
        while (nodes[(root << 1)] >= 0)
        {
            root = nodes[(root << 1)];
        }

        /* 计算有几棵并查树，巧妙利用取反避免重复计算 */
        if (nodes[(root << 1) + RANK] >= 0)
        {
            nodes[(root << 1) + RANK] = ~nclasses++;
        }
        labels[i] = ~nodes[(root << 1) + RANK];
    }

    return nclasses;
}

/*
Function process:
+ Clustered and merge the proposals list of rects.
*/
void clusteringRect(const ResultRect *srcRects, const int srcCount, const float eps, ResultRect *dstRects, int *dstCount)
{
    const int treeNodeCount = srcCount >> 3;
    int index = 0;
    int cls = 0;
    int classesNum = 0;
    float factor = 0;
    ResultRect resultRect;
    int *treeNode = NULL;
    int *labelsNode = NULL;

    treeNode = (int*)malloc((treeNodeCount << 1) * sizeof(int));
    memset(treeNode, 0, (treeNodeCount << 1) * sizeof(int));

    labelsNode = (int*)malloc(treeNodeCount * sizeof(int));
    memset(labelsNode, 0, treeNodeCount * sizeof(int));

    classesNum = clusteredRects(srcRects, srcCount, eps, treeNode, labelsNode);

    *dstCount = classesNum;
    memset(dstRects, 0, classesNum * sizeof(ResultRect));

    for (index = 0; index < srcCount; index++)
    {
        cls = labelsNode[index];
        dstRects[cls].x += srcRects[index].x;
        dstRects[cls].y += srcRects[index].y;
        dstRects[cls].width += srcRects[index].width;
        dstRects[cls].height += srcRects[index].height;
        dstRects[cls].confidence++;
    }
    for (index = 0; index < classesNum; index++)
    {
        resultRect = dstRects[index];
        factor = 1.0f / dstRects[index].confidence;
        dstRects[index].x = myRound(resultRect.x * factor);
        dstRects[index].y = myRound(resultRect.y * factor);
        dstRects[index].width = myRound(resultRect.width * factor);
        dstRects[index].height = myRound(resultRect.height * factor);
    }

    free(treeNode);
    treeNode = NULL;
    free(labelsNode);
    labelsNode = NULL;
}
