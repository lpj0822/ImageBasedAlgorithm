#include "kalmanfilter.h"

/*
 * @brief
 *   Init fields of structure @KalmanOneState.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1;
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 *   state - Klaman filter structure
 *   initX - initial x state value
 *   initP - initial estimated error convariance
 * @outputs
 * @retval
 */
void initOneKalmanFilter(KalmanOneState *state, const float initX, const float initP)
{
    state->x = initX;
    state->p = initP;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */2e2
    state->r = 5e2;//10e-5;  /* measure error convariance */
}

/*
 * @brief
 *   1 Dimension Kalman filter
 * @inputs
 *   state - Klaman filter structure
 *   measureZ - Measure value
 * @outputs
 * @retval
 *   Estimated result
 */
float oneKalmanFilter(KalmanOneState *state, const float measureZ)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (measureZ - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/*
 * @brief
 *   Init fields of structure @KalmanRectState.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0};
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 * @outputs
 * @retval
 */
void initRectKalmanFilter(KalmanRectState *state, const Rectangle rect, const float initP[4])
{
   int i ;
   int initRect[4] = {rect.x, rect.y, rect.width, rect.height};

   state->A = 1;
   state->H = 1;

   for (i =0 ;i< 4;i++)
   {
       state->x[i] = (float)initRect[i];
       state->p[i] = initP[i];
       state->q[i] = 2e2;//10e-6;  /* predict noise convariance */
       state->r[i] = 5e2;//10e-5;  /* measure error convariance */
   }

}


/*
Function process:
    + Do the Kalman filter for state->x
    Fan-in :
            + mvGroupGenerate()
    Fan-out:
            + N/A
    ATTENTION: __________
*/
Rectangle rectKalmanFilter(KalmanRectState *state, const Rectangle measureRect)
{
    int i;
    Rectangle result;
    int measureZ[4] = {measureRect.x, measureRect.y, measureRect.width, measureRect.height};

    /* Predict */
    //for (i =0 ;i< 4;i++)
    //{
    //	state->x[i] = state->A * state->x[i];
    //	state->p[i] = state->A * state->A * state->p[i] + state->q[i];  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    //	/* Measurement */
    //	state->gain[i] = state->p[i] * state->H / (state->p[i] * state->H * state->H + state->r[i]);
    //	state->x[i] = state->x[i] + state->gain[i] * (z_measure[i] - state->H * state->x[i]);
    //	state->p[i] = (1 - state->gain[i] * state->H) * state->p[i];
    //}


    /* Predict，consider A，H= I*/
    for(i = 0; i < 4 ; i++)
    {
        //state->x[i] = state->x[i];
        state->p[i] =  state->p[i] + state->q[i];  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

        /* Measurement */
        state->gain[i] = state->p[i] / (state->p[i]  + state->r[i]);
        state->x[i] = state->x[i] + state->gain[i] * (measureZ[i] -  state->x[i]);
        state->p[i] = (1 - state->gain[i]) * state->p[i];

    }


    result.x = (int)state->x[0];
    result.y = (int)state->x[1];
    result.width = (int)state->x[2];
    result.height = (int)state->x[3];

    return result;
}
