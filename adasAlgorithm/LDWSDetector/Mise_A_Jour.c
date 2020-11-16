#include "data.h"
#include "Matrice.h"

/*
Function process:
	+ Update M_I_new->X and M_I_new->CX based on the detected Zone_detect
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
*/
void M_A_J_Zone(const Modele_Image *M_I_init, Modele_Image *M_I_new, const Zone *Zone_detect, 
				Fichier *Donnees, int indice_zone)
{

	int i, j, k;
	double K;					/* gain of kalman */
	double coeff = 0;

	/*
	 * Copy R
	 */
	for (i = 0; i < 2; i++)
	{
		k				= (indice_zone << 1) + i;
		M_I_new->Xi[k]	= Zone_detect->X[i];
		M_I_new->CXi[k] = Zone_detect->CX[i];
	}

	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		M_I_new->V[j] = M_I_init->V[j];
	}

	/* 2.3.3 Updating
	 * (Hd*Cxd(p-1)*Hd_T + R)^-1
	 */
	coeff = 1 / (M_I_init->CX[indice_zone * (LDWS_NB_ZONES + 1)] + Zone_detect->CX[0]);

	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		/* Kd = Cxd*HdT * ((Hd*Cxd(p-1)*HdT + R)^-1)*/
		K = (M_I_init->CX[indice_zone + LDWS_NB_ZONES * j]) * coeff;

        /* Xd = Xd(p-1) + Kd*[y - x]*/
		Donnees->M_A_J_Zone_X[j] = M_I_init->X[j] + K * (Zone_detect->X[0] - M_I_init->X[indice_zone]);
		for (i = 0; i < LDWS_NB_ZONES; i++)
		{
            /* Cxd = Cxd(p-1) - KdHdCxd(p-1) */
			k = i + j * LDWS_NB_ZONES;
			Donnees->M_A_J_Zone_CX[k] = M_I_init->CX[k] - (M_I_init->CX[indice_zone * LDWS_NB_ZONES + i]) * K;
		}
	}

	indice_zone++;

	coeff = 1 / (Donnees->M_A_J_Zone_CX[indice_zone * (LDWS_NB_ZONES + 1)] + Zone_detect->CX[1]);

	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		K = (Donnees->M_A_J_Zone_CX[indice_zone + LDWS_NB_ZONES * j]) * coeff;
		M_I_new->X[j] = Donnees->M_A_J_Zone_X[j] + K * (Zone_detect->X[1] - Donnees->M_A_J_Zone_X[indice_zone]);
		for (i = 0; i < LDWS_NB_ZONES; i++)
		{
			k = i + j * LDWS_NB_ZONES;
			M_I_new->CX[k] = Donnees->M_A_J_Zone_CX[k] - (Donnees->M_A_J_Zone_CX[indice_zone * LDWS_NB_ZONES + i]) * K;
		}
	}
}

/*
Function process:
	+ update M_3D_est->X.CX.Param.C_param based on the detected M_I_est
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void M_A_J_Param(Modele_3D *M_3D_est,const Modele_3D *M_3D_init, const Modele_Image *M_I_est, Fichier *Donnees)
{
	int i, j, k, t;
	double K;
	double coeff;
	int indice_zone;

	for (j = 0; j < Donnees->NB_Z_P; j++)
	{
		Donnees->M_A_J_Param_X_new[j] = M_3D_init->X[j];
		for (i = 0; i < Donnees->NB_Z_P; i++)
		{
			k = i + j * Donnees->NB_Z_P;
			Donnees->M_A_J_Param_CX_new[k] = M_3D_init->CX[k];
		}
	}
	// 3.1 Localization
	for (k = 0; k < LDWS_NB_ZONES; k++)
	{							/* for each X */
		if (M_I_est->Zones_Detectees[k])
		{
			indice_zone = k;

            /* (H*Cx0*HT + Cxdk)^-1 */
			coeff = 1 / (Donnees->M_A_J_Param_CX_new[indice_zone * (Donnees->NB_Z_P + 1)] + M_I_est->CXi[(k << 1)]);

			for (j = 0; j < Donnees->NB_Z_P; j++)
			{
                /* K = Cx0*HT*coeff */
				K = (Donnees->M_A_J_Param_CX_new[indice_zone + (Donnees->NB_Z_P) * j]) * coeff;

                /* Xk = X0 - k(Xdk - HX0) */
				Donnees->M_A_J_Param_X[j] =
					Donnees->M_A_J_Param_X_new[j] + K * (M_I_est->Xi[(k << 1)] - Donnees->M_A_J_Param_X_new[indice_zone]);
				for (i = 0; i < Donnees->NB_Z_P; i++)
				{
					t = i + j * Donnees->NB_Z_P;
					Donnees->M_A_J_Param_CX[t] = Donnees->M_A_J_Param_CX_new[t]
											   - (Donnees->M_A_J_Param_CX_new[indice_zone * Donnees->NB_Z_P + i]) * K;
				}
			}

			indice_zone++;

			coeff = 1 / (Donnees->M_A_J_Param_CX[indice_zone * (Donnees->NB_Z_P + 1)] + M_I_est->CXi[(k << 1) + 1]);

			for (j = 0; j < Donnees->NB_Z_P; j++)
			{
				K = (Donnees->M_A_J_Param_CX[indice_zone + (Donnees->NB_Z_P) * j]) * coeff;
				Donnees->M_A_J_Param_X_new[j] =
					Donnees->M_A_J_Param_X[j] + K * (M_I_est->Xi[2 * k + 1] - Donnees->M_A_J_Param_X[indice_zone]);
				for (i = 0; i < Donnees->NB_Z_P; i++)
				{
					t = i + j * Donnees->NB_Z_P;
					Donnees->M_A_J_Param_CX_new[t] =
						Donnees->M_A_J_Param_CX[t] - (Donnees->M_A_J_Param_CX[indice_zone * (Donnees->NB_Z_P) + i]) * K;
				}
			}
		}
	}

	/*
	 * copie de CX dans modele est 
	 */
	for (j = 0; j < Donnees->NB_Z_P; j++)
	{
		M_3D_est->X[j] = Donnees->M_A_J_Param_X_new[j];
		for (i = 0; i < Donnees->NB_Z_P; i++)
		{
			M_3D_est->CX[j * Donnees->NB_Z_P + i] = Donnees->M_A_J_Param_CX_new[j * Donnees->NB_Z_P + i];
		}
	}
	for (j = LDWS_NB_ZONES; j < Donnees->NB_Z_P; j++)
	{
		M_3D_est->Param[j - LDWS_NB_ZONES] = Donnees->M_A_J_Param_X_new[j];
		for (i = LDWS_NB_ZONES; i < Donnees->NB_Z_P; i++)
		{
			M_3D_est->C_Param[(j - LDWS_NB_ZONES) * LDWS_NB_PARAM + (i -LDWS_NB_ZONES)] =
				Donnees->M_A_J_Param_CX_new[j * (Donnees->NB_Z_P) + i];
		}
	}
}

/*
Function process:
	+ Tracking based on parameters
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + N/A
	ATTENTION: if line changed, update M_3D_est->Param[1]; else, update M_3D_est->X and M_3D_est->CX based on tracking, 
	update the Init param in next frame M_I_init->X and M_I_init->CX
*/
void M_A_J_Suivi3(Modele_3D *M_3D_est, const Modele_3D *M_3D_init,
			 Modele_Image *M_I_init, Modele_Filter *M_Filter, const Modele_Image *M_I_est, Fichier *Donnees)
{
	int i, j;

	Donnees->changeLine = 0;

	/*
	 * Change lane 
	 */
	if (M_3D_est->Param[1] < -Donnees->ValidWidth)
	{
		M_3D_est->Param[1] = M_3D_est->Param[1] + M_3D_est->Param[0];	
		Donnees->changeLine = 1;
	}
	else if (M_3D_est->Param[1] > M_3D_est->Param[0] + Donnees->ValidWidth)
	{
		M_3D_est->Param[1] = M_3D_est->Param[1] - M_3D_est->Param[0];
		Donnees->changeLine = 2;
	}

	/*
	 * evolution 3D  3.2 tracking
	 */
	for (j = 0; j < LDWS_NB_PARAM; j++)
	{
		for (i = 0; i < LDWS_NB_PARAM; i++)
		{
			Donnees->M_A_J_Suivi3_M_Suivi[i + j * LDWS_NB_PARAM] = 0;
		}
	}

	M_identite(LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_M);
	Donnees->M_A_J_Suivi3_M[LDWS_NB_PARAM + 2] = Donnees->Lambda_Y;

	/*
	 *  M[LDWS_NB_PARAM+3]=Donnees->Lambda_Y*Donnees->Lambda_Y/2;
	 *  M[2*LDWS_NB_PARAM+3]=Donnees->Lambda_Y;
	 */
	TransposeA(Donnees->M_A_J_Suivi3_M, LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_M_T);

	/*
	 * Q test error 
	 */
	Donnees->M_A_J_Suivi3_M_Suivi[0 * LDWS_NB_PARAM + 0] = Donnees->Delta_L * Donnees->Delta_L;
	Donnees->M_A_J_Suivi3_M_Suivi[1 * LDWS_NB_PARAM + 1] = Donnees->Delta_X0 * Donnees->Delta_X0;
	Donnees->M_A_J_Suivi3_M_Suivi[2 * LDWS_NB_PARAM + 2] = Donnees->Delta_Psi * Donnees->Delta_Psi;
	Donnees->M_A_J_Suivi3_M_Suivi[3 * LDWS_NB_PARAM + 3] = Donnees->Delta_Alpha * Donnees->Delta_Alpha;
	Donnees->M_A_J_Suivi3_M_Suivi[4 * LDWS_NB_PARAM + 4] = Donnees->Delta_Ch * Donnees->Delta_Ch;
	Donnees->M_A_J_Suivi3_M_Suivi[5 * LDWS_NB_PARAM + 5] = Donnees->Delta_Cl * Donnees->Delta_Cl;

	/*
	 * X_new = M * Xlk 
	 */
	ProduitAB(Donnees->M_A_J_Suivi3_M, M_3D_est->Param, LDWS_NB_PARAM,
			  LDWS_NB_PARAM, 1, Donnees->M_A_J_Suivi3_X_new);

	/*
	 * CX_new = M * Clk *MT 
	 */
	ProduitAB(M_3D_est->C_Param, Donnees->M_A_J_Suivi3_M_T,
			  LDWS_NB_PARAM, LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_CX_temp);
	ProduitAB(Donnees->M_A_J_Suivi3_M, Donnees->M_A_J_Suivi3_CX_temp,
			  LDWS_NB_PARAM, LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_CX_new);

	/*
	 * CX_new = M * Clk *MT + Q 
	 */
	SommeAB(Donnees->M_A_J_Suivi3_CX_new, Donnees->M_A_J_Suivi3_M_Suivi,
			LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_CX_new);

	/*
	 * filtre de kalman 
	 */
	for (j = 0; j < Donnees->NB_Z_P; j++)
	{
		for (i = 0; i < LDWS_NB_PARAM; i++)
		{
			Donnees->M_A_J_Suivi3_H_T[j + i * Donnees->NB_Z_P] = 0;
		}
	}
	for (i = 0; i < LDWS_NB_PARAM; i++)
	{
		Donnees->M_A_J_Suivi3_H_T[(i + LDWS_NB_ZONES) * LDWS_NB_PARAM + i] = 1;
	}
    TransposeA(Donnees->M_A_J_Suivi3_H_T, Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_H);

    /* AfficheMatrice(H_T, Donnees->NB_Z_P, LDWS_NB_PARAM, "H"); */

	/*
	 * kalman 
	 */
	/* K_inv = H * CX0 * H.T() + CXp */
	ProduitAB(M_3D_init->CX, Donnees->M_A_J_Suivi3_H_T, Donnees->NB_Z_P,
			  Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_Temp);
	ProduitAB(Donnees->M_A_J_Suivi3_H, Donnees->M_A_J_Suivi3_Temp,
			  LDWS_NB_PARAM, Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_CX_temp);
	SommeAB(Donnees->M_A_J_Suivi3_CX_temp, Donnees->M_A_J_Suivi3_CX_new,
			LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_K_inv);
	InverseA(Donnees->M_A_J_Suivi3_K_inv, LDWS_NB_PARAM);

    /* K = CX0 * H.T() * K_inv.Inverse() */
	ProduitAB(Donnees->M_A_J_Suivi3_H_T, Donnees->M_A_J_Suivi3_K_inv,
			  Donnees->NB_Z_P, LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_Temp);
	ProduitAB(M_3D_init->CX, Donnees->M_A_J_Suivi3_Temp, Donnees->NB_Z_P,
			  Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->M_A_J_Suivi3_K);

    /* X0 = X0 + (K * (Xp - H * X0)) */
	ProduitAB(Donnees->M_A_J_Suivi3_H, M_3D_init->X, LDWS_NB_PARAM,
			  Donnees->NB_Z_P, 1, Donnees->M_A_J_Suivi3_X_temp);
	DifferenceAB(Donnees->M_A_J_Suivi3_X_new, Donnees->M_A_J_Suivi3_X_temp,
				 LDWS_NB_PARAM, 1, Donnees->M_A_J_Suivi3_X_temp2);
	ProduitAB(Donnees->M_A_J_Suivi3_K, Donnees->M_A_J_Suivi3_X_temp2,
			  Donnees->NB_Z_P, LDWS_NB_PARAM, 1, Donnees->M_A_J_Suivi3_X);
	SommeAB(M_3D_init->X, Donnees->M_A_J_Suivi3_X, Donnees->NB_Z_P, 1, M_3D_est->X);

	/* CX0 = (Id - K*H) * CX0 */
	ProduitAB(Donnees->M_A_J_Suivi3_H, M_3D_init->CX, LDWS_NB_PARAM,
			  Donnees->NB_Z_P, Donnees->NB_Z_P, Donnees->M_A_J_Suivi3_Temp);
	ProduitAB(Donnees->M_A_J_Suivi3_K, Donnees->M_A_J_Suivi3_Temp,
			  Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->NB_Z_P, Donnees->M_A_J_Suivi3_CX);
	DifferenceAB(M_3D_init->CX, Donnees->M_A_J_Suivi3_CX, Donnees->NB_Z_P, Donnees->NB_Z_P, M_3D_est->CX);

	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		M_I_init->X[j]				= M_3D_est->X[j];
		M_I_init->templet[i].fAngle = M_I_est->templet[i].fAngle;
		for (i = 0; i < LDWS_NB_ZONES; i++)
		{
			M_I_init->CX[i + j * LDWS_NB_ZONES] = M_3D_est->CX[j * Donnees->NB_Z_P + i];
		}
	}

	if( Donnees->changeLine )
	{
		for (i = 0; i < LDWS_NB_ZONES; ++i)
		{
 			M_Filter->dataSrc[i].dataX = M_I_init->X[i];
		}		
	}

}
