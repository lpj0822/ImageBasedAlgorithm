#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>

#define OPENMP 0
#define NEON 0
#define NEON_ASM 0
#define DEBUG 0

#if NEON == 1
#include <arm_neon.h>
#endif


#ifndef assert

#define assert(condition)  \
{\
    if(condition)\
         ;\
        else\
        {\
            printf("\n[ERROR]Assert failed: %s,line %u\n", __FILE__, __LINE__);\
            while (1);\
        }\
}

#endif

int intMin(const int num1, const int num2);
int intMax(const int num1, const int num2);
float floatMin(const float num1, const float num2);
float floatMax(const float num1, const float num2);

#define MY_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))
//#define My_MIN(num1, num2) ((num1) > (num2) ? (num2):(num1))
//#define My_MAX(num1, num2) ((num1) > (num2) ? (num1):(num2))
#define myRound(value)		(int)((value) + ( (value) >= 0 ? 0.5 : -0.5))

#define ALIGN_16BYTE(x) ((unsigned char*)(((unsigned long long)(x) + 15) >> 4 << 4))

#endif // UTILITY_H

