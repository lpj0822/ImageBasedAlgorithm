#include "data.h"
#include "Points.h"
#include "LDWS_Interface.h"

#ifdef _WIN32_LDWS_DEBUG_

	#include "LDWS_Debug.h"
#endif

static double GetFuzzy(double test, double ref);

/************************************************************************************/

/*
Function process:
	+ Give the similarity of test and ref
	Fan-in : 
	        + PointsCandidats()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static double GetFuzzy(double test, double ref)
{
	double ret = 0.0;

	if (test >= 0.5 * ref && test <= ref)
	{
		ret = (test - ref) / ref + 1.0;
	}
	else if (test > ref && test <= 1.5 * ref)
	{
		ret = (ref - test) / ref + 1.0; 
	}
	else
	{
		double ref_tmp = ref * 3.0;

		if (test >= 2.5 * ref && test <= ref_tmp)
		{
			ret = (ref_tmp - test) / ref;
		}
		else if (test > ref_tmp && test <= 3.15 * ref)
		{
			ret = (ref - test) / ref + 1.0; // (-1.15) - (-1)
		}
	}

	return ret;
}

/*
Function process:
	Fan-in : 
	        + Detection_Zone()
	Fan-out:
	        + GetFuzzy()
	ATTENTION: __________
*/
int PointsCandidats(const int ugsup, const int udsup, const int vsup, const int uginf, const int udinf, const int vinf, 
					int *tab_x, int *tab_y, Zone *Zone_detect, const Fichier *Donnees, const unsigned char *Tab_Image)
{
	int ugfonc_inf, udfonc_inf, ugfonc_sup, udfonc_sup;
	double A_d = 0, B_d = 0, A_g = 0, B_g = 0;

    int l, sumGrad, nbPts = 0;

	int Point_g[LDWS_MAX_ROI_HEIGHT];
	int Point_d[LDWS_MAX_ROI_HEIGHT];

	double fAngleSum, fYSum;

	if (ugsup < Donnees->Marge_Image + Donnees->L_Bande_Sup)
	{
		ugfonc_sup = Donnees->Marge_Image;
	}
	else
	{
		ugfonc_sup = ugsup - Donnees->L_Bande_Sup;
	}

	if (udsup > Donnees->Tx - Donnees->Marge_Image - Donnees->L_Bande_Sup)
	{
		udfonc_sup = Donnees->Tx - Donnees->Marge_Image;
	}
	else
	{
		udfonc_sup = udsup + Donnees->L_Bande_Sup;
	}

	if (uginf < Donnees->Marge_Image + Donnees->L_Bande_Inf)
	{
		ugfonc_inf = Donnees->Marge_Image;
	}
	else
	{
		ugfonc_inf = uginf - Donnees->L_Bande_Inf;
	}

	if (udinf > Donnees->Tx - Donnees->Marge_Image - Donnees->L_Bande_Inf)
	{
		udfonc_inf = Donnees->Tx - Donnees->Marge_Image;
	}
	else
	{
		udfonc_inf = udinf + Donnees->L_Bande_Inf;
	}

	if (udfonc_sup != udfonc_inf)
	{
		A_d = (double)(vsup - vinf) / (double)(udfonc_sup - udfonc_inf);
		B_d = (double)(vinf * udfonc_sup - vsup * udfonc_inf) / (double)(udfonc_sup - udfonc_inf);
	}
	else
	{
		A_d = 0;
	}

	if (ugfonc_sup != ugfonc_inf)
	{
		A_g = (double)(vsup - vinf) / (double)(ugfonc_sup - ugfonc_inf);
		B_g = (double)(vinf * ugfonc_sup - vsup * ugfonc_inf) / (double)(ugfonc_sup - ugfonc_inf);
	}
	else
	{
		A_g = 0;
	}

	for (l = 0; l < vinf - vsup; l += Zone_detect->sampleK)
	{
		if (A_g == 0)
		{
			Point_g[l] = ugfonc_sup;
		}
		else
		{
			Point_g[l] = (int)((l + vsup - B_g) / A_g);
		}

		if (A_d == 0)
		{
			Point_d[l] = udfonc_sup;
		}
		else
		{
			Point_d[l] = (int)((l + vsup - B_d) / A_d);
		}

		if (Point_g[l] < Donnees->Marge_Image)
		{
			Point_g[l] = Donnees->Marge_Image;
		}

		if (Point_d[l] > (Donnees->Tx - Donnees->Marge_Image))
		{
			Point_d[l] = Donnees->Tx - Donnees->Marge_Image;
		}
	}

	fAngleSum 	= 0.0;
	fYSum 		= 0.0;
	sumGrad 	= 0;
	//#pragma omp parallel for reduction(+:fAngleSum, fYSum, sumGrad)
	for (l = vsup * Donnees->Tx; l < vinf * Donnees->Tx; l += Zone_detect->sampleK * Donnees->Tx)
	{
		int i, j;
		int maxLeftEdge, maxRightEdge;
#ifndef _NEON_
		int Somme_sup = 0;
#endif
		int Somme_inf = 0;
		int tmpk, tmpTmpLeft, tmpTmpRight;
		int iCnt, iPtr, indexL, core_length, indexLSC, indexLAC, indexLC, indexLPtr;
		double fAngleTmp, sumY, sumG, fAngle, fAngleF = 0;

		// wang add in 20180130
		int edgeNum = 0;
		int xStar = 0, xEnd = 0;
		int xStarUpdata = 0, xEndUpdata = 0;
		int iPtrmax = -1, iPtrmin = INVAILD_TAN, iCntmax = -1, iCntmin = INVAILD_TAN;
		int iPtrF = -1, iCntF = -1, maxLeftEdgeF = 0, maxRightEdgeF = 0, maxLeftY = 0, maxRightY = 0;


#ifdef _NEON_
        int iPtr_Neon = 0;

        unsigned char neon_core[8];

        uint8x8_t ldata_neon;
        uint8x8_t rdata_neon;
        uint16x4_t tempL;
        uint16x4_t tempR;
        int16x4_t sub_neon_x;
        int16x4_t sub_neon_y;
        int16x4_t tempResult;
        int32x2_t result;

#endif

		core_length = Donnees->Grad_Core_Length;

		maxLeftEdge	 = 0;
		maxRightEdge = 0;
		iPtr   = -1;
		iCnt   = -1;
		fAngle = INVAILD_TAN;
		indexLPtr = l / Donnees->Tx;
		indexL    = indexLPtr - vsup;
		indexLSC  = l - core_length * Donnees->Tx;
		indexLAC  = l + core_length * Donnees->Tx;
		indexLC   = l - core_length;

#ifdef _NEON_

        for (i = Point_g[indexL]; i < Point_d[indexL]; i += Zone_detect->sampleK)
        {
            iPtr_Neon = 0;
            for (j = indexLSC + i; j < l + i; j += Donnees->Tx)
            {
                neon_core[iPtr_Neon] = Tab_Image[j];
                iPtr_Neon++;
            }


            ldata_neon = vld1_u8(neon_core);
            iPtr_Neon = 0;
            for (j = l + i; j < indexLAC + i; j += Donnees->Tx)
            {
                neon_core[iPtr_Neon] = Tab_Image[j];
                iPtr_Neon++;
            }
            rdata_neon = vld1_u8(neon_core);
            tempL = vpaddl_u8(ldata_neon);
            tempR = vpaddl_u8(rdata_neon);
            sub_neon_y = vsub_s16(vreinterpret_s16_u16(tempR), vreinterpret_s16_u16(tempL));

            ldata_neon = vld1_u8(Tab_Image + i + indexLC);
            rdata_neon = vld1_u8(Tab_Image + i + l);
            tempL = vpaddl_u8(ldata_neon);
            tempR = vpaddl_u8(rdata_neon);
            sub_neon_x = vsub_s16(vreinterpret_s16_u16(tempR), vreinterpret_s16_u16(tempL));

            tempResult = vpadd_s16(sub_neon_y, sub_neon_x);
            result = vpaddl_s16(tempResult);

            Somme_inf = vget_lane_s32(result, 1);
            fAngleTmp = vget_lane_s32(result, 0);

        	Donnees->edgeValueV[edgeNum] = fAngleTmp;
        	Donnees->edgeValue[edgeNum] = Somme_inf;
        	edgeNum++;

            if (Somme_inf > 0)
            {
                tmpTmpLeft = Somme_inf;
                if (tmpTmpLeft > maxLeftEdge)
                {
                    maxLeftEdge = tmpTmpLeft;
#ifndef _USEPEAKS_
                    if (maxLeftEdge > Zone_detect->measure.iGrad)
                    {
                        iPtr = i;
                        fAngle = fAngleTmp / Somme_inf;
                    }
#endif
                }
            }
            else
            {
                tmpTmpRight = -Somme_inf;
                if (tmpTmpRight > maxRightEdge)
                {
                    maxRightEdge = tmpTmpRight;
#ifndef _USEPEAKS_
                    if (maxRightEdge > Zone_detect->measure.iGrad)
                    {
                        iCnt = i;
                        fAngle = fAngleTmp / Somme_inf;
                    }
#endif
                }
            }
        }
#else
		
		for (i = Point_g[indexL]; i < Point_d[indexL]; i += Zone_detect->sampleK)
		{			
			Somme_sup = 0;
			Somme_inf = 0;
			for (j = indexLSC + i; j < l + i; j += Donnees->Tx)
			{
				Somme_inf += Tab_Image[j];
			}
			for (j = l + i; j < indexLAC + i; j += Donnees->Tx)
			{
				Somme_sup += Tab_Image[j];
			}
			fAngleTmp = Somme_sup - Somme_inf;
			//my_printf("%d: fAngleTmp=%d\n",i,fAngleTmp);
			Donnees->edgeValueV[edgeNum] = fAngleTmp;

			Somme_sup = 0;
			Somme_inf = 0;
			for (j = i + indexLC; j < i + l; j++)
			{
				Somme_inf += Tab_Image[j];
			}
			for (j = i + l; j < i + core_length + l; j++)
			{
				Somme_sup += Tab_Image[j];
			}

			Somme_inf = Somme_sup - Somme_inf;
			//my_printf("%d: Somme_inf=%d\n",i,Somme_inf);
			Donnees->edgeValue[edgeNum] = Somme_inf;
					edgeNum++;

			if (Somme_inf > 0)
			{
				tmpTmpLeft = Somme_inf;
				if (tmpTmpLeft > maxLeftEdge)
				{
					maxLeftEdge = tmpTmpLeft;
#ifndef _USEPEAKS_
					if (maxLeftEdge > Zone_detect->measure.iGrad)
					{
						iPtr = i;
						fAngle = fAngleTmp / Somme_inf;
					}
#endif
				}
			}
			else
			{
				tmpTmpRight = -Somme_inf;
				if (tmpTmpRight > maxRightEdge)
				{
					maxRightEdge = tmpTmpRight;
#ifndef _USEPEAKS_
					if (maxRightEdge > Zone_detect->measure.iGrad)
					{
						iCnt = i;
						fAngle = fAngleTmp / Somme_inf;
					}
#endif
				}
			}
		}
#endif

#ifdef _USEPEAKS_
		xStar = 2;
		xEnd = edgeNum-2;
		while(xStar<edgeNum && xEnd<edgeNum)
		{
			for(i = xStar; i < edgeNum-2; i++)
			{
				if((Donnees->edgeValue[i] > Donnees->edgeValue[i-1]) &&(Donnees->edgeValue[i] > Donnees->edgeValue[i-2]) &&
					(Donnees->edgeValue[i] > Donnees->edgeValue[i+1]) &&(Donnees->edgeValue[i] > Donnees->edgeValue[i+2]) &&
					(Donnees->edgeValue[i] > 0.5*maxLeftEdge) && Donnees->edgeValue[i] > Zone_detect->measure.iGrad)
				{
					xStarUpdata = 1;// find the peak
					xStar = i;
					for(i = xStar; i < edgeNum-2; i++)
					{
						if((Donnees->edgeValue[i] < Donnees->edgeValue[i-1]) &&(Donnees->edgeValue[i] < Donnees->edgeValue[i-2]) &&
						(Donnees->edgeValue[i] < Donnees->edgeValue[i+1]) &&(Donnees->edgeValue[i] < Donnees->edgeValue[i+2]) &&
						(Donnees->edgeValue[i] < - 0.5*maxRightEdge) && ABS(Donnees->edgeValue[i]) > Zone_detect->measure.iGrad)
						{
							xEndUpdata = 1;// find the trough
							xEnd = i;
							fAngle = 0.5*(ABS(Donnees->edgeValueV[xStar] / Donnees->edgeValue[xStar]) + ABS(Donnees->edgeValueV[xEnd] / Donnees->edgeValue[xEnd]));
							iPtr = xStar*Zone_detect->sampleK + Point_g[indexL];
							iCnt = xEnd*Zone_detect->sampleK + Point_g[indexL];

							if (iPtr < iCnt && iPtr > Point_g[indexL] + core_length && iCnt <  Point_d[indexL] - core_length
								&& ((((iCnt - iPtr)>LDWS_GetXLengthofImage(0.08, indexLPtr)) && (iCnt - iPtr)<LDWS_GetXLengthofImage(0.25, indexLPtr))
								|| (((iCnt - iPtr)>LDWS_GetXLengthofImage(0.4, indexLPtr)) && (iCnt - iPtr)<LDWS_GetXLengthofImage(0.5, indexLPtr)))
								&& ABS(fAngle) < MAX_LANE_ANGLE)
							{
								if(Zone_detect->RFlag)
								{
									if((iPtr < iPtrmin) && (Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)] > 0.8*maxRightY))
									{										
										iPtrmin = iPtr;
										iCntmin = iCnt;
										iPtrF = iPtrmin;
										iCntF = iCntmin;
										fAngleF = fAngle;
										maxLeftEdgeF = Donnees->edgeValue[xStar];
										maxRightEdgeF = - Donnees->edgeValue[xEnd];
										maxRightY = Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)] > maxRightY	? Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)]:maxRightY;															
									}
								}
								else
								{
									if(iPtr > iPtrmax && (Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)] > 0.8*maxLeftY))
									{
										iPtrmax = iPtr;
										iCntmax = iCnt;
										iPtrF = iPtrmax;
										iCntF = iCntmax;
										fAngleF = fAngle;
										maxLeftEdgeF = Donnees->edgeValue[xStar];
										maxRightEdgeF = - Donnees->edgeValue[xEnd];
										maxLeftY = Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)] > maxLeftY ? Tab_Image[l + (int)((iPtr + iCnt) * 0.5 + 0.5)]:maxLeftY;
									}
								}

								/*printf("MidMaxLeft = %d(%d); MidMaxRight = %d(%d);\n", Donnees->edgeValue[xStar],iPtr,\
								Donnees->edgeValue[xEnd],iCnt);*/

							}

							xEndUpdata = 0;
							xStarUpdata = 0;
							break;
						}
					}
				}
				//printf("%d: %d \n", i, edgeValue[i]);
			}
			if(xStarUpdata == 0 || xEndUpdata == 0)
			{
				//printf("can not find max or min!\n");
 				break;
			}

		}

		if( iPtrF >0 && iCntF >0)
		{
			sumY = 0;
			sumG = 0;
			for (i = iPtrF + l; i < iCntF + l; ++i)
			{
				sumY += Tab_Image[i];
			}
			tmpk = (int)((iPtrF + iCntF) * 0.5 + 0.5);
			sumY = sumY / (iCntF - iPtrF);
			sumG = (maxLeftEdgeF + maxRightEdgeF)>>1;

			if(sumY > Zone_detect->measure.fYAve && sumG > Zone_detect->measure.iGrad)
			{
				fAngleSum += fAngleF;
				fYSum     += sumY;
				sumGrad   += sumG;
				/*printf("MaxLeft = %d(%d); MaxRight = %d(%d);\n", maxLeftEdgeF,iPtrF,\
									maxRightEdgeF,iCntF);*/
	//#pragma omp critical	
				{
					tab_x[nbPts] = tmpk;
					tab_y[nbPts] = indexLPtr;
					nbPts++;
				}

#ifdef _WIN32_LDWS_DEBUG_
			if (Donnees->Debug)
			{
			Affiche_Points(ugfonc_inf, udfonc_inf,ugfonc_sup,udfonc_sup, vsup, vinf,\
							iPtrF,indexLPtr, 1);
			Affiche_Points(ugfonc_inf, udfonc_inf,ugfonc_sup,udfonc_sup, vsup, vinf,\
							iCntF,indexLPtr, 1);
			}
#endif // end of _WIN32_LDWS_DEBUG_
			}
		}

#else
		if (iPtr != -1 && iCnt != -1 && iPtr < iCnt
			&& iPtr > Point_g[indexL] + core_length && iCnt <  Point_d[indexL] - core_length
			&&((iCnt - iPtr) < LDWS_GetXLengthofImage(0.2, indexLPtr)) && ((iCnt - iPtr) > LDWS_GetXLengthofImage(0.08, indexLPtr))
			&& ABS(fAngle) < MAX_LANE_ANGLE)//&& GetFuzzy(iCnt - iPtr + 0.0, Donnees->kWidth * 0.15 * (indexLPtr - Donnees->Cy)) >DELTA_LANE_WIDTH_WIDE
		{
			sumY = 0;
			for (i = iPtr + l; i < iCnt + l; ++i)
			{
				sumY += Tab_Image[i];
			}
			tmpk = (int)((iPtr + iCnt) * 0.5 + 0.5);

			fAngleSum += fAngle;
			fYSum     += sumY / (iCnt - iPtr);
			sumGrad   += maxLeftEdge + maxRightEdge;

			{
				tab_x[nbPts] = tmpk;
				tab_y[nbPts] = indexLPtr;
				nbPts++;
			}

#ifdef _WIN32_LDWS_DEBUG_
			if (Donnees->Debug)
			{
			    Affiche_Points(ugfonc_inf, udfonc_inf,ugfonc_sup,udfonc_sup, vsup, vinf,\
									iPtrF,indexLPtr, 1);
				Affiche_Points(ugfonc_inf, udfonc_inf,ugfonc_sup,udfonc_sup, vsup, vinf,\
									iCntF,indexLPtr, 1);
			}
#endif

		}
#endif

	}

	if (nbPts != 0)
	{
		fYSum /= nbPts;
		if (fYSum >= Zone_detect->measure.fYAve)
		{
			Zone_detect->measure.fYAve	= fYSum;
			Zone_detect->measure.fAngle = fAngleSum / nbPts;
			Zone_detect->measure.iGrad	= sumGrad / nbPts;
		}
		else
		{
			nbPts = 0;
		}
	}

	return (nbPts);
}

/*
Function process:
	Fan-in : 
	        + Detection_Zone_LR()
	Fan-out:
	        + GetFuzzy()
	ATTENTION: __________
*/
int PointsCandidats_LR(const int ugsup, const int udsup, const int vsup, const int uginf, const int udinf,const int vinf,
					   int *tab_x, int *tab_y, Zone *Zone_detect, const Fichier *Donnees, const unsigned char *Tab_Image)
{
	int ugfonc_inf, udfonc_inf, ugfonc_sup, udfonc_sup;
	double A_d = 0, B_d = 0, A_g = 0, B_g = 0;

    int l, sumGrad, nbPts = 0;

	int Point_g[LDWS_MAX_ROI_HEIGHT];
	int Point_d[LDWS_MAX_ROI_HEIGHT];

	double fAngleSum, fYSum;

	if (ugsup < Donnees->Marge_Image + Donnees->L_Bande_Sup)
	{
		ugfonc_sup = Donnees->Marge_Image;
	}
	else
	{
		ugfonc_sup = ugsup - Donnees->L_Bande_Sup;
	}
	if (udsup > Donnees->Tx - Donnees->Marge_Image - Donnees->L_Bande_Sup)
	{
		udfonc_sup = Donnees->Tx - Donnees->Marge_Image;
	}
	else
	{
		udfonc_sup = udsup + Donnees->L_Bande_Sup;
	}

	if (uginf < Donnees->Marge_Image + Donnees->L_Bande_Inf)
	{
		ugfonc_inf = Donnees->Marge_Image;
	}
	else
	{
		ugfonc_inf = uginf - Donnees->L_Bande_Inf;
	}
	if (udinf > Donnees->Tx - Donnees->Marge_Image - Donnees->L_Bande_Inf)
	{
		udfonc_inf = Donnees->Tx - Donnees->Marge_Image;
	}
	else
	{
		udfonc_inf = udinf + Donnees->L_Bande_Inf;
	}

	if (udfonc_sup != udfonc_inf)
	{
		A_d = (double)(vsup - vinf) / (double)(udfonc_sup - udfonc_inf);	/* */
		B_d = (double)(vinf * udfonc_sup - vsup * udfonc_inf) / (double)(udfonc_sup - udfonc_inf);
	}
	else
	{
		A_d = 0;
	}

	if (ugfonc_sup != ugfonc_inf)
	{
		A_g = (double)(vsup - vinf) / (double)(ugfonc_sup - ugfonc_inf);
		B_g = (double)(vinf * ugfonc_sup - vsup * ugfonc_inf) / (double)(ugfonc_sup - ugfonc_inf);
	}
	else
	{
		A_g = 0;
	}

	for (l = 0; l < vinf - vsup; l += Zone_detect->sampleK)
	{
		if (A_g == 0)
		{
			Point_g[l] = ugfonc_sup;
		}
		else
		{
			Point_g[l] = (int)((l + vsup - B_g) / A_g);
		}
		if (A_d == 0)
		{
			Point_d[l] = udfonc_sup;
		}
		else
		{
			Point_d[l] = (int)((l + vsup - B_d) / A_d);
		}
		if (Point_g[l] < Donnees->Marge_Image)
		{
			Point_g[l] = Donnees->Marge_Image;
		}
		if (Point_d[l] > (Donnees->Tx - Donnees->Marge_Image))
		{
			Point_d[l] = Donnees->Tx - Donnees->Marge_Image;
		}
	}

	fAngleSum 	= 0.0;
	fYSum 		= 0.0;
	sumGrad 	= 0;

//#pragma omp parallel for reduction(+:fAngleSum, fYSum, sumGrad)
	for (l = vsup * Donnees->Tx; l < vinf * Donnees->Tx; l += Zone_detect->sampleK * Donnees->Tx)
	{
		int i, j;
		int maxLeftEdge, maxRightEdge;
#ifndef _NEON_
		int Somme_sup = 0;
#endif
		int Somme_inf = 0;
		int tmpk, tmpTmpLeft, tmpTmpRight;
		int iCnt, iPtr, indexL, core_length, indexLSC, indexLAC, indexLC, indexLPtr;
		double fAngleTmp, sumY, fAngle;

#ifdef _NEON_

        int iPtr_Neon = 0;

        unsigned char neon_core[8];

        uint8x8_t ldata_neon;
        uint8x8_t rdata_neon;
        uint16x4_t tempL;
        uint16x4_t tempR;
        int16x4_t sub_neon_x;
        int16x4_t sub_neon_y;
        int16x4_t tempResult;
        int32x2_t result;

#endif

		core_length = Donnees->Grad_Core_Length;

		maxLeftEdge	 = 0;
		maxRightEdge = 0;
		iPtr   = -1;
		iCnt   = -1;
		fAngle = INVAILD_TAN;
		indexLPtr = l / Donnees->Tx;
		indexL    = indexLPtr - vsup;
		indexLSC  = l - core_length * Donnees->Tx;
		indexLAC  = l + core_length * Donnees->Tx;
		indexLC   = l - core_length;

#ifdef _NEON_

        for (i = Point_g[indexL]; i < Point_d[indexL]; i += Zone_detect->sampleK)
        {
            iPtr_Neon = 0;
            for (j = indexLSC + i; j < l + i; j += Donnees->Tx)
            {
                neon_core[iPtr_Neon] = Tab_Image[j];
                iPtr_Neon++;
            }
            ldata_neon = vld1_u8(neon_core);
            iPtr_Neon = 0;
            for (j = l + i; j < indexLAC + i; j += Donnees->Tx)
            {
                neon_core[iPtr_Neon] = Tab_Image[j];
                iPtr_Neon++;
            }
            rdata_neon = vld1_u8(neon_core);
            tempL = vpaddl_u8(ldata_neon);
            tempR = vpaddl_u8(rdata_neon);
            sub_neon_y = vsub_s16(vreinterpret_s16_u16(tempR), vreinterpret_s16_u16(tempL));

            ldata_neon = vld1_u8(Tab_Image + i + indexLC);
            rdata_neon = vld1_u8(Tab_Image + i + l);
            tempL = vpaddl_u8(ldata_neon);
            tempR = vpaddl_u8(rdata_neon);
            sub_neon_x = vsub_s16(vreinterpret_s16_u16(tempR), vreinterpret_s16_u16(tempL));

            tempResult = vpadd_s16(sub_neon_y, sub_neon_x);
            result = vpaddl_s16(tempResult);

            Somme_inf = vget_lane_s32(result, 1);
            fAngleTmp = vget_lane_s32(result, 0);

            if (Somme_inf > 0)
            {
                tmpTmpLeft = Somme_inf;
                if (tmpTmpLeft > maxLeftEdge)
                {
                    maxLeftEdge = tmpTmpLeft;
                    if (maxLeftEdge > Zone_detect->measure.iGrad)
                    {
                        iPtr = i;
                        fAngle = fAngleTmp / Somme_inf;
                    }
                }
            }
            else
            {
                tmpTmpRight = -Somme_inf;
                if (tmpTmpRight > maxRightEdge)
                {
                    maxRightEdge = tmpTmpRight;
                    if (maxRightEdge > Zone_detect->measure.iGrad)
                    {
                        iCnt = i;
                        fAngle = fAngleTmp / Somme_inf;
                    }
                }
            }
        }
#else
		
		for (i = Point_g[indexL]; i < Point_d[indexL]; i += Zone_detect->sampleK)
		{			
			Somme_sup = 0;
			Somme_inf = 0;
			for (j = indexLSC + i; j < l + i; j += Donnees->Tx)
			{
				Somme_inf += Tab_Image[j];
			}
			for (j = l + i; j < indexLAC + i; j += Donnees->Tx)
			{
				Somme_sup += Tab_Image[j];
			}
			fAngleTmp = Somme_sup - Somme_inf;
			//my_printf("%d: fAngleTmp=%d\n",i,fAngleTmp);

			Somme_sup = 0;
			Somme_inf = 0;
			for (j = i + indexLC; j < i + l; j++)
			{
				Somme_inf += Tab_Image[j];
			}
			for (j = i + l; j < i + core_length + l; j++)
			{
				Somme_sup += Tab_Image[j];
			}

			Somme_inf = Somme_sup - Somme_inf;
			//my_printf("%d: Somme_inf=%d\n",i,Somme_inf);

			if (Somme_inf > 0)
			{
				tmpTmpLeft = Somme_inf;
				if (tmpTmpLeft > maxLeftEdge)
				{
					maxLeftEdge = tmpTmpLeft;
					if (maxLeftEdge > Zone_detect->measure.iGrad)
					{
						iPtr = i;
						fAngle = fAngleTmp / Somme_inf;
					}
				}
			}
			else
			{
				tmpTmpRight = -Somme_inf;
				if (tmpTmpRight > maxRightEdge)
				{
					maxRightEdge = tmpTmpRight;
					if (maxRightEdge > Zone_detect->measure.iGrad)
					{
						iCnt = i;
						fAngle = fAngleTmp / Somme_inf;
					}
				}
			}
		}
#endif

		if (iPtr != -1 && iCnt != -1 && iPtr < iCnt
			&& iPtr > Point_g[indexL] + core_length && iCnt <  Point_d[indexL] - core_length
			&& GetFuzzy(iCnt - iPtr + 0.0, Donnees->kWidth * 0.15 * (indexLPtr - Donnees->Cy)) >
			DELTA_LANE_WIDTH_WIDE && ABS(fAngle) < MAX_LANE_ANGLE)
		{
			sumY = 0;
			for (i = iPtr + l; i < iCnt + l; ++i)
			{
				sumY += Tab_Image[i];
			}
			tmpk = (int)((iPtr + iCnt) * 0.5 + 0.5);

			fAngleSum += fAngle;
			fYSum     += sumY / (iCnt - iPtr);
			sumGrad   += maxLeftEdge + maxRightEdge;

			{
				tab_x[nbPts] = tmpk;
				tab_y[nbPts] = indexLPtr;
				nbPts++;
			}

#ifdef _WIN32_LDWS_DEBUG_
			if (Donnees->Debug)
			{
			Affiche_Points(tab_x[nbPts - 1], tab_y[nbPts - 1], Donnees->Aff_Median);
			}
#endif
		}
	}

	if (nbPts != 0)
	{
		fYSum /= nbPts;
		if (fYSum >= Zone_detect->measure.fYAve)
		{
			Zone_detect->measure.fYAve	= fYSum;
			Zone_detect->measure.fAngle = fAngleSum / nbPts;
			Zone_detect->measure.iGrad	= sumGrad / (nbPts * 2);
		}
		else
		{
			nbPts = 0;
		}
	}

	return (nbPts);
}

