#include "data.h"
#include "LDWS_Interface.h"
#include "Bspline.h"

#define CB_K		(0.166667)

static void CBSpline(Point *p, int srclen, LDWS_Point *pLPoint, int *pLlen);

static double CBbase(int x, double t);

/************************************************************************************/

static double CBbase(int x, double t)
{
	switch (x)
	{
        case 0:
            /* (-t * t * t + 3.0 * t * t - 3.0 * t + 1.0 ) / 6.0 */
			return (-t * t * t * CB_K + 0.5 * t * t - 0.5 * t + CB_K);
			break;
		case 1:
            /* (3.0 * t * t * t - 6.0 * t * t + 4.0 ) / 6.0 */
			return (0.5 * t * t * t - t * t + 0.666667);
			break;
		case 2:
             /* (-3.0 * t * t * t + 3.0 * t * t + 3.0 * t + 1.0 ) / 6.0 */
			return (-0.5 * t * t * t + 0.5 * t * t + 0.5 * t + CB_K);
			break;
		case 3:
			return (t * t * t * CB_K);
			break;
		default:
			return (0.0);
	}

	return (0.0);
}

static void CBSpline(Point *p, int srclen, LDWS_Point *pLPoint, int *pLlen)
{
	int k, m, t;
	double x;
	double y;
	double temp;

	*pLlen = 0;
	for (m = 0; m < srclen - 3; ++m)
	{
		for (t = 0; t < LDWS_OUTPUT_CBP_NUM; ++t)
		{
			x = 0.0;
			y = 0.0;
			for (k = 0; k < 4; ++k)
			{
				temp	= (double)t / LDWS_OUTPUT_CBP_NUM;
				x		+= p[m + k].x * CBbase(k, temp);
				y		+= p[m + k].y * CBbase(k, temp);
			}
			pLPoint[*pLlen].x		= (int)(x + 0.5);
			pLPoint[(*pLlen)++].y	= (int)(y + 0.5);
		}
	}
}

void CBsplineFilter(const Modele_Image *M_I_est, LDWS_Output *L_output, Fichier *Donnees)
{
	int iLPtr = 0, iPPtr, iLPPtr = 0;
	int tempLPtr, incTempLPtr, n, m, k;

	for (; iLPtr < LDWS_NB_BANDES; ++iLPtr)
	{
		iPPtr		= 0;
		tempLPtr	= iLPtr * Donnees->NB_INTERVALLES;
		k			= iLPtr * L_output->LPointLength;
		incTempLPtr = tempLPtr + 1;

		Donnees->pCBPoint[iPPtr].x = M_I_est->X[tempLPtr] * 2.0 - M_I_est->X[incTempLPtr];
		Donnees->pCBPoint[iPPtr].y = (M_I_est->V[tempLPtr] << 1) - M_I_est->V[incTempLPtr];
		for (; iPPtr < CLIP_BSPLINE_POINT_NUM; ++iPPtr)
		{
			n = iPPtr + 1;
			m = tempLPtr + iPPtr;
			Donnees->pCBPoint[n].x = M_I_est->X[m];
			Donnees->pCBPoint[n].y = M_I_est->V[m];
		}

		CBSpline(Donnees->pCBPoint, CLIP_BSPLINE_POINT_NUM + 1, &L_output->pPoint[k], &iLPPtr);
		for (; iPPtr < Donnees->NB_INTERVALLES; ++iPPtr, iLPPtr++)
		{
			n = k + iLPPtr;
			m = tempLPtr + iPPtr;
			L_output->pPoint[n].x = (int)(M_I_est->X[m] + 0.5);
			L_output->pPoint[n].y = M_I_est->V[m];
		}
	}
}
