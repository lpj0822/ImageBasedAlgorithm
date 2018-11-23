#include "quick_sort.h"
#include <stdio.h>
#include <stdlib.h>

#include "utility.h"

#define DEPTH 1024

static void swap(int *num1, int *num2)
{
    int tmp = *num1;
    *num1 = *num2;
    *num2 = tmp;
}

static int partition(int *array, const int length)
{
    int i = 0;
    int pivot = 0;
    // swap random slot selection to end.
    //array[length-1] will hold the pivot value.
    swap(array + rand() % length, array + length - 1);
    for (i = 0; i < length; i++)
    {
        if (array[i] < array[length-1])
        {
            swap(array + i, array + pivot);
            pivot++;
        }
    }
    // swap the pivot value into position
    swap(array + pivot, array + length - 1);
    return pivot;
}

//quick sort recursive implementation
void quickSort(int* array, const int count)
{
#if DEBUG == 1
    assert(array != NULL);
#endif
    if (count < 2)
        return;
    int pivot = partition(array, count);
    quickSort(array, pivot);
    pivot += 1;//skips pivot slot
    quickSort(array + pivot, count - pivot);
}

//quick sort non-recursive implementation
void quickSortNonRecursive(int *array, const int count)
{
#if DEBUG == 1
    assert(array != NULL);
#endif
    int pivot = 0;
    int begin[DEPTH] = {0};
    int end[DEPTH] = {0};
    int i = 0;
    int leftIndex = 0;
    int rightIndex = 0;
    int tempSwap = 0;
    begin[0] = 0;
    end[0] = count;
    while (i>=0)
    {
        leftIndex = begin[i];
        rightIndex = end[i] - 1;
        if (leftIndex < rightIndex)
        {
            pivot = array[leftIndex];
            while (leftIndex < rightIndex)
            {
                while (array[rightIndex] >= pivot &&  leftIndex < rightIndex)
                    rightIndex--;
                if (leftIndex < rightIndex)
                {
                    array[leftIndex] = array[rightIndex];
                    leftIndex++;
                }
                while (array[leftIndex] <= pivot && leftIndex < rightIndex)
                    leftIndex++;
                if (leftIndex < rightIndex)
                {
                    array[rightIndex] = array[leftIndex];
                    rightIndex--;
                }
            }
            array[leftIndex] = pivot;
            begin[i+1] = leftIndex + 1;
            end[i+1] = end[i];
            end[i] = leftIndex;
            i++;
            if (end[i] - begin[i] > end[i-1] - begin[i-1])
            {
                tempSwap = begin[i];
                begin[i] = begin[i-1];
                begin[i-1] = tempSwap;
                tempSwap = end[i];
                end[i] = end[i-1];
                end[i-1] = tempSwap;
            }
        }
        else
            i--;
    }
}
