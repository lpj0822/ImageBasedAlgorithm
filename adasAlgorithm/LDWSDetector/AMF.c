#include "data.h"
#include "AMF.h"
#include "LDWS_Interface.h"
#include "Init_Struct.h"
#include <string.h>
#include <stdio.h>

static int frameKeep = 0;

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	diffX		          double*		      differents between ref and cur.
[in]	    ref		              double	          Input value.
[in/out]	cur		              double*	          Filtered value.
[in]        ratio                 double              ratio value
[in]	    asymptotic		      double	          range of AMF.
[in]	    shift		          double	          shift value

Realized function:
    + Adaptive median filter(AMF) for cur
*/
static void AMF(double *diffX, double ref, double *cur, double ratio, double asymptotic, double shift);

/**********************************************************************************************************/

/*
Function process:
	+ If line detected Copy M_I_est to M_Filter; else init M_Filter
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void AMF_Copy(const Modele_Image *M_I_est, Modele_Filter *M_Filter,const Fichier *Donnees)
{
	int i;

	if (M_I_est->Route == 1 && ((Donnees->Confidence_detection[0] > Donnees->Confidence - KEEP_FRAME) || (Donnees->Confidence_detection[1] > Donnees->Confidence - KEEP_FRAME )))
	{
		for (i = 0; i < LDWS_NB_ZONES; ++i)
		{
			M_Filter->dataSrc[i].dataX = M_I_est->X[i];
			M_Filter->dataSrc[i].dataA = M_I_est->templet[i].fAngle;
		}
		M_Filter->detect_flg = 1;
		M_Filter->Route = M_I_est->Route;
	}
	else
	{
		Init_Modele_Filter(M_Filter, Donnees);
	}
}

/*
Function process:
	+ Do AFM filter for param
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void AMF_Param(double *diffX, double ref, double *cur, double ratio, Fichier *Donnees, const int i)
{
	switch (i)
	{
		case 0:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_L, Donnees->Delta_L);
			break;
		case 1:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_X0, Donnees->Delta_X0);
			break;
		case 2:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_Alpha, Donnees->Delta_Alpha);
			break;
		case 3:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_Psi, Donnees->Delta_Psi);
			break;
		case 4:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_Ch, Donnees->Delta_Ch);
			break;
		case 5:
			AMF(diffX, ref, cur, ratio, Donnees->Delta_Cl, Donnees->Delta_Cl);
			break;
		default:
			break;
	}
}

/*
Function process:
	+ Adaptive median filter(AMF) for cur
	Fan-in : 
	        + AMF_Pro()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static void AMF(double *diffX, double ref, double *cur, double ratio, double asymptotic, double shift)
{
	if (*diffX < asymptotic && *diffX > -asymptotic)
	{
		*cur = ref * (1 - ratio) + *cur * ratio;
	}
	else if (*diffX >= asymptotic)
	{
		*cur	= ref + shift;
		*diffX	= shift;
	}
	else if (*diffX <= -asymptotic)
	{
		*cur	= ref - shift;
		*diffX	= -shift;
	}
}

/*
Function process:
	+ Adaptive median filter(AMF) for M_I_est->X
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void AMF_Pro(Modele_Image *M_I_est, Modele_Filter *M_Filter, Fichier *Donnees)
{
	double temp;
	int i, j, iCnt, iCntTmp;
	Templet sumTemplet;

	for (i = 0; i < LDWS_NB_BANDES; ++i)
	{
		memset(&sumTemplet, 0, sizeof(Templet));
		for (j = 0; j < Donnees->NB_INTERVALLES; ++j)
		{
			iCnt = i * Donnees->NB_INTERVALLES + j;
			if (M_I_est->Zones_Detectees[iCnt] == 1)
			{
				sumTemplet.iGrad += M_I_est->templet[iCnt].iGrad;
				sumTemplet.fYAve += M_I_est->templet[iCnt].fYAve;
			}
			else
			{
				sumTemplet.iGrad += Donnees->templet[iCnt].iGrad;
				sumTemplet.fYAve += Donnees->templet[iCnt].fYAve;
			}
		}
		sumTemplet.iGrad /= Donnees->NB_INTERVALLES;
		sumTemplet.fYAve /= Donnees->NB_INTERVALLES;
		for (j = 0; j < Donnees->NB_INTERVALLES; ++j)
		{
			iCnt							= i * Donnees->NB_INTERVALLES + j;
			Donnees->templet[iCnt].iGrad	= (int)(((sumTemplet.iGrad + Donnees->templet[iCnt].iGrad) >> 1) + 0.5);
			Donnees->templet[iCnt].fYAve	= (int)((sumTemplet.fYAve + Donnees->templet[iCnt].fYAve) * 0.5 + 0.5);
		}
	}

	Donnees->flgStatus = ROAD_NORMAL;

 	if (M_Filter->detect_flg == 1)
	{
		iCnt	= 0;
		iCntTmp = 0;
		memset(Donnees->dataAve, 0, LDWS_NB_BANDES * sizeof(Data_Stat));
		memset(Donnees->dataDiff, 0, LDWS_NB_ZONES * sizeof(Data_Stat));

		for (i = 0; i < LDWS_NB_ZONES; ++i)
		{
			Donnees->dataDiff[i].dataX = M_I_est->X[i] - M_Filter->dataSrc[i].dataX;
			Donnees->dataDiff[i].dataA = M_I_est->templet[i].fAngle - M_Filter->dataSrc[i].dataA;
			if (ABS(Donnees->dataDiff[i].dataX) < Donnees->fInterFrameDisp[i])
			{
				iCnt++;
			}
			if (M_I_est->Zones_Detectees[i] == 1)
			{
				Donnees->dataAve[i / Donnees->NB_INTERVALLES].dataX += Donnees->dataDiff[i].dataX;
				Donnees->dataAve[i / Donnees->NB_INTERVALLES].dataA += Donnees->dataDiff[i].dataA;
				iCntTmp++;
			}
		}

		if (iCnt >= Donnees->NB_INTERVALLES && iCntTmp != 0)
		{
			for (i = 0; i < LDWS_NB_BANDES; ++i)
			{
				Donnees->dataAve[i].dataX /= iCntTmp;
				Donnees->dataAve[i].dataA /= iCntTmp;
				if (M_Filter->dataAveDiff[i].dataA != INVAILD_TAN)
				{
					temp = Donnees->dataAve[i].dataA - M_Filter->dataAveDiff[i].dataA;
					AMF(&temp, M_Filter->dataAveDiff[i].dataA,
							&Donnees->dataAve[i].dataA, 0.5, DELTA_ANGLE_FRAME, DELTA_ANGLE_FRAME);
				}
				M_Filter->dataAveDiff[i].dataA = Donnees->dataAve[i].dataA;;
				if (M_Filter->dataAveDiff[i].dataX != INVAILD_TAN)
				{
					temp = Donnees->dataAve[i].dataX - M_Filter->dataAveDiff[i].dataX;
					AMF(&temp, M_Filter->dataAveDiff[i].dataX,
							&Donnees->dataAve[i].dataX, 0.5, DELTA_X_FRAME, DELTA_X_FRAME);
				}
				M_Filter->dataAveDiff[i].dataX = Donnees->dataAve[i].dataX;
			}

			if (M_I_est->Route == 1)
			{
				frameKeep = 0;
				for (i = 0; i < LDWS_NB_ZONES; ++i)
				{
					temp = ABS(M_Filter->dataAveDiff[i / Donnees->NB_INTERVALLES].dataX) * 0.04 + 0.2;
					if (temp > 0.5)
					{
						temp = 0.5;
					}
					M_I_est->X[i] = M_Filter->dataSrc[i].dataX + Donnees->dataDiff[i].dataX * temp;
				}
			}
			else
			{
				if ((++frameKeep <= KEEP_FRAME) && (M_Filter->Route ==1))
				{
					for (i = 0; i < LDWS_NB_ZONES; ++i)
					{
						temp = ABS(M_Filter->dataAveDiff[i / Donnees->NB_INTERVALLES].dataX) * 0.025 + 0.25;
						if (temp > 0.5)
						{
							temp = 0.5;
						}
						M_I_est->X[i] = M_Filter->dataSrc[i].dataX
									  + Donnees->dataDiff[i].dataX * temp * (KEEP_FRAME + 1.0 - frameKeep)
									  / KEEP_FRAME;
					}
					M_I_est->Route = M_Filter->Route;
				}
			}
		}
		else
		{
                        printf("Change too much between two frames! \n");
			Init_Modele_Filter(M_Filter, Donnees);
			memset(Donnees->dataAve, 0, LDWS_NB_BANDES * sizeof(Data_Stat));
			memset(Donnees->dataDiff, 0, LDWS_NB_ZONES * sizeof(Data_Stat));
			Donnees->flgStatus				= ROAD_CHANGE;
			Donnees->Confidence_detection[0]	= 0;
		}
	}

	if (M_I_est->Route && Donnees->flgStatus == ROAD_NORMAL)
	{
		for (i = 0; i < Donnees->NB_INTERVALLES; ++i)
		{
			if (M_I_est->X[i] >= M_I_est->X[i + Donnees->NB_INTERVALLES]
				- Donnees->kWidth * 0.15 * (M_I_est->V[i] - Donnees->Cy))
			{
                                printf("Two lines crossing! \n");
				M_I_est->Route = 0;
			}
		}
	}

	if (frameKeep != 0 || M_I_est->Route == 0)
	{
		for (i = 0; i < LDWS_NB_ZONES; ++i)
		{
			Donnees->templet[i].iGrad = (int)(DOWN_TH_GRAD * Donnees->templet[i].iGrad + 0.5);
			if (Donnees->templet[i].iGrad < Donnees->Seuil_Grad)
			{
				Donnees->templet[i].iGrad = Donnees->Seuil_Grad;
			}
			Donnees->templet[i].fYAve *= DOWN_TH_GRAD;
			if (Donnees->templet[i].fYAve < MIN_LING_AVE_Y)
			{
				Donnees->templet[i].fYAve = MIN_LING_AVE_Y;
			}
		}
	}
}
