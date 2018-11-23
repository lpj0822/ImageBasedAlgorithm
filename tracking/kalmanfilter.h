#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "baseAlgorithm/image_data_structure.h"

/*
 * NOTES: n Dimension means the state is n dimension,
 * measurement always 1 dimension
 */

/* 1 Dimension */
typedef struct KalmanOneState{
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} KalmanOneState;



/* Rect Dimension{x,y,w,h} */
typedef struct KalmanRectState
{
    float x[4];  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q[4];  /* process(predict) noise convariance */
    float r[4];  /* measure noise convariance */
    float p[4];  /* estimated error convariance */
    float gain[4];
} KalmanRectState;

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          KalmanOneState*		      KalmanOneState.
[in]	    initX		          const float		                  init value for state->x.
[in]	    initP		          const float		                  init value for state->p.

Realized function:
    + init the values for initOneKalmanFilter for one Dimension.
*/
 void initOneKalmanFilter(KalmanOneState *state, const float initX, const float initP);

 /*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          KalmanOneState*		      KalmanOneState.
[in]	    measureZ		      const float		                  measurement for x.

Realized function:
    + Do the Kalman filter for x based on the measurement z.
*/
float oneKalmanFilter(KalmanOneState *state, const float measureZ);

/*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          KalmanRectState*		      KalmanRectState.
[in]	    rect		          const Rectangle		              init value for state->x.
[in]	    initP		          const float[4]		              init value for state->p.

Realized function:
    + init the values for initRectKalmanFilter for four Dimension.
*/
void initRectKalmanFilter(KalmanRectState *state, const Rectangle rect, const float initP[4]);

 /*
I/O:	    Name		          Type	     		          Content

[in/out]	state		          KalmanRectState*		      KalmanRectState.
[in]	    measureRect		      const Rectangle		              measurement for x.

Realized function:
    + Do the Kalman filter for x based on the measurement z.
*/
Rectangle rectKalmanFilter(KalmanRectState *state, const Rectangle measureRect);

#endif // KALMANFILTER_H
