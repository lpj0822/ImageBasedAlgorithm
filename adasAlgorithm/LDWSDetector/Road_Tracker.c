#include <math.h>
#include <string.h>
#include <stdio.h>
#include "data.h"
#include "LDWS_Interface.h"
#include "Init_Struct.h"
#include "Initialisation.h"
#include "Recherche.h"
#include "Mise_A_Jour.h"
#include "AMF.h"
#include "Bspline.h"

#ifdef _WIN32_LDWS_DEBUG_

	#include "LDWS_Debug.h"
#endif
static int frameKeepL = 0;
static int frameKeepR = 0;
static int frameKeepLL = 0;
static int frameKeepRR = 0;

/*
Function process:
	+ Init the values for parameters
	Fan-in : 
	        + LDWS_Init()
	Fan-out:
	        + Get_Fichier()
			+ Init_Modele_Fichier()
			+ Allocation_Modele_Struct()
			+ Init_Modele_3D()
			+ Init_Modele_Output()
			+ Init_Modele_Filter()
			+ Initialisation()

	ATTENTION: __________
*/
void LDWS_InitRoadTracker(int mode, 
						  Modele_Image *M_I_init, Modele_Image *M_I_est,
						  Modele_3D *M_3D_init, Modele_3D *M_3D_est,
						  Modele_Filter *M_Filter, LDWS_Output *L_output, 
						  Fichier *Donnees, const char *fichier_init,const char *fichier_init_custom)
{
	int i = 0;
        printf("%d",i);
    /* read param */
	if (mode == STATIQUE)
	{
		Get_Fichier(Donnees, fichier_init, fichier_init_custom);
	}
#ifdef _LDWS_detail_DEBUG_
	my_printf("VEHICULE=%d \n", Donnees->VEHICULE);
	my_printf("MARQUEE=%d \n", Donnees->MARQUEE);

	my_printf("Eu=%d \n", Donnees->Eu);
	my_printf("Ev=%d \n", Donnees->Ev);

	my_printf("Tx=%d \n", Donnees->Tx);
	my_printf("Ty=%d \n", Donnees->Ty);

	my_printf("Cx=%d \n", Donnees->Cx);
	my_printf("Cy=%d \n", Donnees->Cy);

	my_printf("Z0=%lf \n", Donnees->Z0);

	my_printf("Largeur=%lf \n", Donnees->Largeur);
	my_printf("X0=%lf \n", Donnees->X0);
	my_printf("Alpha=%lf \n", Donnees->Alpha);
	my_printf("Psi=%lf \n", Donnees->Psi);
	my_printf("Ch=%lf \n", Donnees->Ch);
	my_printf("Cl=%lf \n", Donnees->Cl);

	my_printf("Dev_Largeur=%lf \n", Donnees->Dev_Largeur);
	my_printf("Dev_X0=%lf \n", Donnees->Dev_X0);
	my_printf("Dev_Alpha=%lf \n", Donnees->Dev_Alpha);
	my_printf("Dev_Psi=%lf \n", Donnees->Dev_Psi);
	my_printf("Dev_Ch=%lf \n", Donnees->Dev_Ch);
	my_printf("Dev_Cl=%lf \n", Donnees->Dev_Cl);

	my_printf("V_haut=%d \n", Donnees->V_haut);
	my_printf("V_bas=%d \n", Donnees->V_bas);
	my_printf("H_zone=%d \n", Donnees->H_zone);

	my_printf("Var_Max=%d \n", Donnees->Var_Max);
	my_printf("Var_Min=%d \n", Donnees->Var_Min);
	my_printf("Largeur_Zone_Min=%d \n", Donnees->Largeur_Zone_Min);
	my_printf("Largeur_Zone_Max=%d \n", Donnees->Largeur_Zone_Max);
	my_printf("Marge_Image=%d \n", Donnees->Marge_Image);
	my_printf("Nb_Pts_Min=%d \n", Donnees->Nb_Pts_Min);

	my_printf("Seuil_Grad=%d \n", Donnees->Seuil_Grad);
	my_printf("Grad_Core_Length=%d \n", Donnees->Grad_Core_Length);
	my_printf("L_Bande_Sup=%d \n", Donnees->L_Bande_Sup);
	my_printf("L_Bande_Inf=%d \n", Donnees->L_Bande_Inf);

	my_printf("Nb_Tir_Med=%d \n", Donnees->Nb_Tir_Med);
	my_printf("Confiance=%lf \n" , Donnees->Confiance);
	my_printf("Ecart_Type=%lf \n", Donnees->Ecart_Type);
	my_printf("Residu_Max=%d \n", Donnees->Residu_Max);
	my_printf("Coef_Median=%lf \n", Donnees->Coef_Median);
	my_printf("Conf_Pts=%d \n", Donnees->Conf_Pts);
	my_printf("Erreur_mini=%lf \n", Donnees->Erreur_mini);

	my_printf("Nb_Iterations=%d \n", Donnees->Nb_Iterations);
	my_printf("Proba=%lf \n", Donnees->Proba);
	my_printf("Proba_Bord=%lf \n", Donnees->Proba_Bord);
	my_printf("Tolerance=%lf \n", Donnees->Tolerance);

	my_printf("Lambda_Y=%lf \n", Donnees->Lambda_Y);
	my_printf("Delta_L=%lf \n", Donnees->Delta_L);
	my_printf("Delta_X0=%lf \n", Donnees->Delta_X0);
	my_printf("Delta_Alpha=%lf \n", Donnees->Delta_Alpha);
	my_printf("Delta_Psi=%lf \n", Donnees->Delta_Psi);
	my_printf("Delta_Ch=%lf \n", Donnees->Delta_Ch);
	my_printf("Delta_Cl=%lf \n", Donnees->Delta_Cl);

	my_printf("Confidence=%d \n", Donnees->Confidence);

	my_printf("ValidWidth=%lf \n", Donnees->ValidWidth);
	my_printf("Filter_cp=%d \n", Donnees->Filter_cp);

    /* for Debug */
	my_printf("Debug=%d \n", Donnees->Debug);
	my_printf("Aff_Zone=%d \n", Donnees->Aff_Zone);
	my_printf("Aff_Median=%d \n", Donnees->Aff_Median);
	my_printf("Aff_Result=%d \n", Donnees->Aff_Result);
#endif

	Init_Modele_Fichier(Donnees);

#ifdef _LDWS_detail_DEBUG_
	my_printf("NB_Z_P=%d \n", Donnees->NB_Z_P);
	my_printf("NB_INTERVALLES=%d \n", Donnees->NB_INTERVALLES);
#endif

    /* allocation all structure */
	Allocation_Modele_Struct(M_3D_init, M_3D_est, M_Filter, Donnees);

	Init_Modele_3D(M_I_init, M_3D_init, NULL, Donnees);

	Init_Modele_Output(L_output, Donnees, M_3D_init);

	Init_Modele_Filter(M_Filter, Donnees);

	Initialisation(M_I_init, M_I_est, M_3D_init, Donnees);

	for (i = 0; i < LDWS_NB_ZONES; ++i)
	{
		L_output->pCaPoint[i].y = M_I_init->V[i];
#ifdef _LDWS_detail_DEBUG_
		my_printf("M_I_init->V[%d]=%d \n",i,M_I_init->V[i]);
#endif
		L_output->pCaPoint[i].x = (int)(M_I_init->X[i] + 0.5);
#ifdef _LDWS_detail_DEBUG_
		my_printf("M_I_init->X[%d]=%f \n",i,M_I_init->X[i]);
#endif
	}

	Donnees->Init = 1;
}

/*
Function process:
	+ The main realization of LDWS
	Fan-in : 
	        + LDWS_Tracker()
	Fan-out:
	        + Init_Modele_Image()
			+ Recherche()
			+ AMF_Pro()
			+ AMF_Copy()
			+ M_A_J_Param()
			+ AMF_Param()
			+ CBsplineFilter()
			+ M_A_J_Suivi3()
			+ LDWS_ChangeResultWtoN()

	ATTENTION: __________
*/
void LDWS_RoadTracker(Modele_Image *M_I_init, Modele_Image *M_I_est, 
					  Modele_3D *M_3D_init, Modele_3D *M_3D_est,
					  Modele_Filter *M_Filter, LDWS_Output *L_output,
					  Fichier *Donnees, const unsigned char *Tab_Image)
{
	int i = 0;
	int RouteFlg = 0;
	//int k1,k2;

//    my_printf("%d %d\n", frameKeepL,frameKeepR);

	Donnees->Arbre = 0;
	
	Init_Modele_Image(M_I_init, M_I_est, Donnees);

	Donnees->kWidth = Donnees->k * cos(L_output->Param[3]);

	for (i = 0; i < LDWS_NB_ZONES; ++i)
	{
		Donnees->fInterFrameDisp[i] = Donnees->kWidth * 0.2 * (M_I_est->V[i] - Donnees->Cy);
	}
	
	Recherche(M_I_init, M_I_est, Donnees, Tab_Image);



#if 1
    //my_printf("[WS LDWS] route : %d\n", M_I_est->Route);
	if (M_I_est->Route == 1)
	{
		if (++Donnees->Confidence_detection[0] > Donnees->Confidence)
		{
			Donnees->Confidence_detection[0] = Donnees->Confidence;
		}
		RouteFlg = 1;
	}
	else
	{
		if (--Donnees->Confidence_detection[0] < 0)
		{
			Donnees->Confidence_detection[0] = 0;
		}
		RouteFlg = 0;
	}
	
	AMF_Pro(M_I_est, M_Filter, Donnees);

	if(M_I_est->Route == 0 && M_I_est->Proba_Bord[0] > 0.15)
		M_I_est->Route =2;

	if(M_I_est->Route == 0 && M_I_est->Proba_Bord[1] > 0.15)
		M_I_est->Route =3;

	if( M_I_est->Route == 0 && M_I_est->Proba_Bord_half[0] > 0.15 && M_I_est->Proba_Bord_half[1] > 0.15)
	{
		M_I_est->Route_half =1;
	}

	if (M_I_est->Route_half == 1 || RouteFlg == 1)
	{
		if (++Donnees->Confidence_detection[1] > Donnees->Confidence)
		{
			Donnees->Confidence_detection[1] = Donnees->Confidence;
		}
	}
	else
	{
		if (--Donnees->Confidence_detection[1] < 0)
		{
			Donnees->Confidence_detection[1] = 0;
		}
	}

	if (M_I_est->Route == 2 || RouteFlg == 1)
	{
		if (++Donnees->Confidence_detection[2] > Donnees->Confidence)
		{
			Donnees->Confidence_detection[2] = Donnees->Confidence;
		}
	}
	else
	{
		if (--Donnees->Confidence_detection[2] < 0)
		{
			Donnees->Confidence_detection[2] = 0;
		}
	}

	if (M_I_est->Route == 3 || RouteFlg == 1)
	{
		if (++Donnees->Confidence_detection[3] > Donnees->Confidence)
		{
			Donnees->Confidence_detection[3] = Donnees->Confidence;
		}
	}
	else
	{
		if (--Donnees->Confidence_detection[3] < 0)
		{
			Donnees->Confidence_detection[3] = 0;
		}
	}
	
	if (M_I_est->Route_half == 1)
		M_I_est->Route = 1;
	
	if(M_I_est->Route==1 && LDWS_NB_BANDES==4 && ~M_I_est->Route_half)
	{
	   //Recherche_LR(M_I_est, M_I_est, Donnees, Tab_Image);
	}

	if (M_I_est->RouteL == 1)
	{
		if (++Donnees->Confidence_detection[4]> Donnees->Confidence)
		{
			Donnees->Confidence_detection[4] = Donnees->Confidence;
		}
	}
	else
	{
		if (--Donnees->Confidence_detection[4] < 0)
		{
			Donnees->Confidence_detection[4] = 0;
		}
	}

	if (M_I_est->RouteR == 1)
	{
		if (++Donnees->Confidence_detection[5] > Donnees->Confidence)
		{
			Donnees->Confidence_detection[5] = Donnees->Confidence;
		}
	}
	else
	{
		if (--Donnees->Confidence_detection[5] < 0)
		{
			Donnees->Confidence_detection[5] = 0;
		}
	}

	if (M_I_est->RouteL == 1)
	{
		frameKeepL = 0;		
	}
	else
	{
		if (++frameKeepL <= KEEP_FRAME)
			M_I_est->RouteL = 1;
	}

	if (M_I_est->RouteR == 1)
	{
		frameKeepR = 0;		
	}
	else
	{
		if (++frameKeepR <= KEEP_FRAME)
			M_I_est->RouteR = 1;
	}

	if (M_I_est->Route == 2)
	{
		frameKeepLL = 0;		
	}
	else if(M_I_est->Route == 0 && M_I_est->Route != 3 && Donnees->Confidence_detection[2] >= Donnees->Confidence - KEEP_FRAME)
	{
		if (++frameKeepLL <= KEEP_FRAME)
			M_I_est->Route = 2;
	}

	if (M_I_est->Route == 3)
	{
		frameKeepRR = 0;		
	}
	else if(M_I_est->Route == 0 && M_I_est->Route != 2 && Donnees->Confidence_detection[3] >= Donnees->Confidence - KEEP_FRAME)
	{
		if (++frameKeepRR <= KEEP_FRAME)
			M_I_est->Route = 3;
	}

	if (M_I_est->Route)
	{
		M_A_J_Param(M_3D_est, M_3D_init, M_I_est, Donnees);
		if(M_I_est->Route == 2)
		{
			if(M_3D_est->Param[1] < 0 || M_3D_est->Param[1] > M_3D_init->Param[1] +1)
				M_I_est->Route = 0;
		}
		if(M_I_est->Route == 3)
		{
			if(M_3D_est->Param[1] < M_3D_init->Param[1] - 1 || M_3D_est->Param[1] > M_3D_init->Param[0])
				M_I_est->Route = 0;
		}
		if(M_I_est->Route > 0 && (M_3D_est->Param[0] > 4.5 || M_3D_est->Param[0] < 2.5))
		{
			M_I_est->Route = 0;		    
		}
	}

	L_output->Route = M_I_est->Route;
	L_output->Route_half = M_I_est->Route_half;
	L_output->Route_L = M_I_est->RouteL;
	L_output->Route_R = M_I_est->RouteR;
	//my_printf("Route=%d  Route_L=%d  Route_R=%d\n",L_output->Route,L_output->Route_L,L_output->Route_R);

	if( Donnees->changeLine == 1)
	{
		 L_output->Param[1] = L_output->Param[1] + L_output->Param[0];
	}
	else if( Donnees->changeLine == 2 )
	{
		L_output->Param[1] = L_output->Param[1] - L_output->Param[0];
	}
	else
	{
	}

	if (M_I_est->Route)
	{
		double temp = 0;

        //my_printf("[WS LDWS] route : %d =============================================================================\n", M_I_est->Route);

		AMF_Copy(M_I_est, M_Filter, Donnees);

		//M_A_J_Param(M_3D_est, M_3D_init, M_I_est, Donnees);//根据M_I_est更新M_3D_est

		for (i = 0; i < LDWS_NB_PARAM; ++i)
		{
			if (M_Filter->detect_flg == 1)
			{
				temp = M_3D_est->Param[i] - L_output->Param[i];
				AMF_Param(&temp, L_output->Param[i], &M_3D_est->Param[i], 0.5, Donnees, i);
			}
			L_output->Param[i] = M_3D_est->Param[i];
		}

		for (i = 0; i < 6; ++i)
		{
		  L_output->Confidence_detection[i] = Donnees->Confidence_detection[i];
		}

		if ((L_output->Confidence_detection[0] > L_output->Confidence - KEEP_FRAME) || (L_output->Confidence_detection[1] > L_output->Confidence - KEEP_FRAME) || \
			(L_output->Confidence_detection[2] > L_output->Confidence - KEEP_FRAME) || (L_output->Confidence_detection[3] > L_output->Confidence - KEEP_FRAME))
		{
			for (i = 0; i < LDWS_NB_BANDES * Donnees->NB_INTERVALLES; ++i)
			{
				L_output->pCaPoint[i].x = (int)(M_I_est->X[i] + 0.5);
			}

			Donnees->Vh = (int)((Donnees->Cy - Donnees->Ev * L_output->Param[3]) * 0.3 + Donnees->Vh * 0.7 + 0.5);
		}
		else
		{
			for (i = 0; i < LDWS_NB_BANDES * Donnees->NB_INTERVALLES; ++i)
			{
				L_output->pCaPoint[i].x = (int)(M_3D_init->X[i] + 0.5);
			}
		}
	
		//CBsplineFilter(M_I_est, L_output, Donnees);

		M_A_J_Suivi3(M_3D_est, M_3D_init, M_I_init,M_Filter, M_I_est, Donnees);
	}
	else
	{
		Init_Modele_3D(M_I_init, M_3D_init, L_output, Donnees);
	}

	//LDWS_ChangeResultWtoN();

	LDWS_CalVanishPointSet();

#endif

}
