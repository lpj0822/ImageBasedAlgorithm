#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define NUM_MAX_LEN  (16)

int myGetInt(char *buf);
double myGetDouble(char *buf);

char myGetc(FILE* fp)
{
    char result;

    fread(&result,1,1,fp);

    return result;
}

int myFScanfInt(FILE *fp, const char *fmt, int *ptr)
{

    unsigned char i = 0;
    char tmp[NUM_MAX_LEN];

    if(fp == NULL)
        return -1;

    while(fread(&tmp[0],1,1,fp) > 0)
    {
        if(tmp[0] >= '0' && tmp[0] <= '9') 
            break;
    }

    for(i = 1; i < NUM_MAX_LEN; i++)
    {
        if(fread(&tmp[i],1,1,fp) <= 0)
            break;
        if('\n' == tmp[i])
            break;
    }
    tmp[i] = '\0';

    *ptr = myGetInt(tmp);

    return 1;
}

int myFScanfDouble(FILE *fp, const char *fmt, double *ptr)
{

    unsigned char i = 0;
    char tmp[NUM_MAX_LEN];

    if(fp == NULL)
        return -1;

    while(fread(&tmp[0],1,1,fp) > 0)
    {
        if(tmp[0] >= '0' && tmp[0] <= '9') 
            break;
    }

    for(i = 1; i < NUM_MAX_LEN; i++)
    {
        if(fread(&tmp[i],1,1,fp) <= 0)
            break;
        if('\n' == tmp[i])
            break;
    }
    tmp[i] = '\0';

    *ptr = myGetDouble(tmp);

    return 1;
}

int myGetInt(char *buf)
{
    int result = 0;
    unsigned char i = 0;
    unsigned char len = strlen(buf);

    if(NULL == buf || len <= 0)
        return 0;

    for(i = 0; i < len-1; i++)
    {
        if(0 > (*(buf+i)-'0')&&(*(buf+i)-'0')>9)
            break;
        result *= 10;
        result += (*(buf+i)-'0');
    }

    return result;
}

double myGetDouble(char *buf)
{
    
    double result = 0;
    unsigned char index = 0;
    unsigned char len = strlen(buf);
    char* pointIndex = strchr(buf,'.');

    if(NULL == buf || len <= 0)
        return 0;

    for(index = 0; index < len-1; index++)
    {

        if(*(buf+index) == '.')
            continue;
        else if(*(buf+index) < '0' && *(buf+index)-'0'> '9')
            break;

        result *= 10;
        result += (*(buf+index) - '0');
    }

    if(pointIndex != NULL)
    {
        for(index = 2; index < len-(pointIndex - buf); index++)
            result /= 10;
    }

    return result;
}

