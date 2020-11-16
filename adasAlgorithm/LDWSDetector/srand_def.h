/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: srand_def.h
Version: 1.0		Date: 2017-04-28		Author: Yanming Wang		ID: 1047930

Description:
	The functions in this file are defined to realize the rand of int.
	The following function types are included:
	+ srand48(): 
	+ drand48(): 

Deviation:

History:
	+ Version: 1.0		Date: 2017-04-28		Author: Yanming Wang	ID: 1047930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef _DRAND48_H_
#define _DRAND48_H_

#include <stdlib.h>
	
#define MNWZ     (0x100000000)
#define ANWZ     (0x5DEECE66D)
#define CNWZ     (0xB16)
	
#define WS_INFINITY (0xFFFFFFFFF)

int labelsize;
int dim;

static unsigned long long seed = 1;

float ld_drand48(void)
{
	unsigned int x;
	seed = (ANWZ * seed + CNWZ) & 0xFFFFFFFFFFFFLL;
	x = (unsigned int)(seed >> 16);
	return ((float)x / (float)MNWZ);
}

void ld_srand48(unsigned int i)
{
	seed = (((long long int)i) << 16) | rand();
} 

#endif	/*  */
