#include <math.h>
#include <stdlib.h>
#include "data.h"
#include "Matrice.h"
#include "LDWS_Interface.h"

#include "srand_def.h"

#ifdef _WIN32_LDWS_DEBUG_

	#include "LDWS_Debug.h"
#endif

/*
Function process:
	+ line fitting based on the detected points, and update Zone_detect->X and Zone_detect->CX
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int Median(int *tab_x, int *tab_y, int *nb_pts, Zone *Zone_detect, Fichier *Donnees)
{
	int i, j;

	double ordonnee = 0, pente = 0, pente_est = 0, erreur_pente = 0;
	int Nb_Tir_Possible = 0;
	int indice_pt1 = 0, indice_pt2 = 1;
	int indice_pt1_fin = 0, indice_pt2_fin = 0;

	double X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;	/* couple de points pour une droite */
	double a, b, c, e;			/* droite --line = ay +bx +c */

	int nb_pts_med = 0;			/* nombre de couple de points quis sont bon */
	int nb_pts_max = 0;	
	int nb_cp_elim = 0;	
	double residu_min = Donnees->Residu_Max * Donnees->Residu_Max;
	double residu_total = 0;
	double median = 0;
	double ecart = 0;

	double denominateur;
#ifdef _NE10_

	ne10_float32_t temp;
	ne10_float32_t temp1;
	ne10_float32_t temp2;
	ne10_mat2x2f_t Jacob_neon, Jacob_T_neon, Cov_pts_neon, Cov_est_neon, Temp_neon;
#else

	double Jacob[4], Jacob_T[4], Cov_pts[4], Cov_est[4], Temp[4];
#endif

	ld_srand48(0);

	pente		 = (Zone_detect->X[1] - Zone_detect->X[0]) / (double)(Zone_detect->V[1] - Zone_detect->V[0]);
	erreur_pente = ABS((1 / (double)((Zone_detect->V[1] - Zone_detect->V[0]) * (Zone_detect->V[1] - Zone_detect->V[0])))
				 * (Zone_detect->CX[0] + Zone_detect->CX[1] - (Zone_detect->CX[2] * 2.0) ));

	if((Zone_detect->indice_zone % Donnees->NB_INTERVALLES) < 2)
		erreur_pente = erreur_pente * 3;

	if (pente == 0 || erreur_pente < Donnees->Erreur_mini)
	{
		erreur_pente = Donnees->Erreur_mini;
	}

	Nb_Tir_Possible = (*nb_pts) * (*nb_pts - 1)/2;	/* possible line number */

	if (Nb_Tir_Possible <= Donnees->Nb_Tir_Med)
    {
        /* possible lane number < 20 */
		for (j = 0; j < Nb_Tir_Possible; j++)
		{
			X1 = (double)tab_x[indice_pt1];
			Y1 = (double)tab_y[indice_pt1];
			X2 = (double)tab_x[indice_pt2];
			Y2 = (double)tab_y[indice_pt2];

			a = (X1 - X2) / (Y1 - Y2);

			/*
			 * si la pente est en dehors dla marge imposee par la pente et l
			 * erreur sur la pente on choisit un nouveau couple de points 
			 */
			if (a < pente + Donnees->Confiance * erreur_pente && a > pente - Donnees->Confiance * erreur_pente)
			{
				e = sqrt((a * a) + 1);
				c = (X1 - a * Y1) / e;
				b = -1.0 / e;
				a /= e;

				nb_pts_med		= 0;
				residu_total	= 0;

				for (i = 0; i < *nb_pts; i++)
				{
					Donnees->tab_residus[i] = (a * tab_y[i] + b * tab_x[i] + c) * (a * tab_y[i] + b * tab_x[i] + c);
					if (Donnees->tab_residus[i] < Donnees->Ecart_Type)
					{
						residu_total += Donnees->tab_residus[i];
						nb_pts_med++; 
					}
				}

				if (residu_min > residu_total && nb_pts_med >= *nb_pts >> 1)
				{
					nb_pts_max		= nb_pts_med;
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}
				else if (nb_pts_med == nb_pts_max && residu_min > residu_total)
				{
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}
				else if (nb_pts_med > nb_pts_max)
				{
					nb_pts_max		= nb_pts_med;
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}

                /* indice du nouveau couple de points */
				if (indice_pt2 == *nb_pts - 1)
				{
					indice_pt1++;
					indice_pt2 = indice_pt1 + 1;
				}
				else
				{
					indice_pt2++;
				}
			}
            else /* elimination du couple de points et indice du nouveau couple de points */
			{
				nb_cp_elim++;
				if (indice_pt2 == *nb_pts - 1)
				{
					indice_pt1++;
					indice_pt2 = indice_pt1 + 1;
				}
				else
				{
					indice_pt2++;
				}
			}
		}
		if (nb_cp_elim == Nb_Tir_Possible)
		{
			return (0);
		}

	}
	else
	{
		for (j = 0; j < Donnees->Nb_Tir_Med; j++)
		{
            /* recherche alatoire des couples de points */
			indice_pt1 = (int)(ld_drand48() * (*nb_pts));
			indice_pt2 = (int)(ld_drand48() * (*nb_pts));

			while (indice_pt1 == indice_pt2)
			{
				indice_pt2 = (int)(ld_drand48() * (*nb_pts));
			}

			X1 = (double)tab_x[indice_pt1];
			Y1 = (double)tab_y[indice_pt1];
			X2 = (double)tab_x[indice_pt2];
			Y2 = (double)tab_y[indice_pt2];

			/* calcul de la pente de la droite */
			a = (X1 - X2) / (Y1 - Y2);

            /*
			 * si la pente est en dehors dla marge imposee par la pente et l
			 * erreur sur la pente on choisit un nouveau couple de points 
			 */
			if (a < pente + Donnees->Confiance * erreur_pente && a > pente - Donnees->Confiance * erreur_pente)
			{
				/* calcul des coefficients de la droite */
				e = sqrt((a * a) + 1);
				c = (X1 - a * Y1) / e;
				b = -1.0 / e;
				a /= e;

				nb_pts_med		= 0;
				residu_total	= 0;

                /* calcul des residus des poiConf_Ptsnts a la droite */
				for (i = 0; i < *nb_pts; i++)
				{
					Donnees->tab_residus[i] = (a * tab_y[i] + b * tab_x[i] + c) * (a * tab_y[i] + b * tab_x[i] + c);
					if (Donnees->tab_residus[i] < Donnees->Ecart_Type)
					{
						residu_total += Donnees->tab_residus[i];
						nb_pts_med++;
					}
				}

                /* sauvegarde du meilleur couple de points */
				if (residu_min > residu_total && nb_pts_med >= *nb_pts >> 1)
				{
					nb_pts_max		= nb_pts_med;
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}
				else if (nb_pts_med == nb_pts_max && residu_min > residu_total)
				{
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}
				else if (nb_pts_med > nb_pts_max)
				{
					nb_pts_max		= nb_pts_med;
					residu_min		= residu_total;
					indice_pt1_fin	= indice_pt1;
					indice_pt2_fin	= indice_pt2;
				}
			}
            else /* elimination du couple de points */
			{
				nb_cp_elim++;
			}
		}
		if (nb_cp_elim == Donnees->Nb_Tir_Med)
		{
			return (0);
		}
	}

	median = residu_min / (double)(nb_pts_max);

	X1 = (double)tab_x[indice_pt1_fin];
	Y1 = (double)tab_y[indice_pt1_fin];
	X2 = (double)tab_x[indice_pt2_fin];
	Y2 = (double)tab_y[indice_pt2_fin];

	pente_est	= (X1 - X2) / (double)(Y1 - Y2);
	ordonnee	= (X1 * Y2 - X2 * Y1) / (Y2 - Y1);

	a = pente_est;
	e = sqrt(a * a + 1);
	c = (X1 - a * Y1) / e;
	b = -1.0 / e;
	a /= e;

	if (median == 0)
	{
		ecart = Donnees->Coef_Median;
	}
	else
	{
		if (Nb_Tir_Possible <= Donnees->Nb_Tir_Med)
		{
			ecart = Donnees->Coef_Median * (1.4826 * (1 + (5.0 / (double)(Nb_Tir_Possible - 2.0))) * sqrt(median));
		}
		else
		{
			ecart = Donnees->Coef_Median * (1.4826 * (1 + (5.0 / (double)(Donnees->Nb_Tir_Med - 2.0))) * sqrt(median));
		}
	}

    j = 0;
	for (i = 0; i < *nb_pts; i++)
	{
		Donnees->tab_residus[i] = (a * tab_y[i] + b * tab_x[i] + c) * (a * tab_y[i] + b * tab_x[i] + c);
		if (Donnees->tab_residus[i] > ecart)
		{
			tab_x[i] = (Donnees->Tx << 1);
			tab_y[i] = (Donnees->Tx << 1);
			j++;
		}
	}

	if (*nb_pts - j < 2)
	{
		return (0);
	}

	Y1 = 0;
	Y2 = Donnees->Ty;
	for (i = 0; i < *nb_pts; i++)
	{
		if (tab_y[i] != (Donnees->Tx << 1))
		{
			if (tab_y[i] > Y1)
			{
				Y1 = tab_y[i];
			}
			if (tab_y[i] < Y2)
			{
				Y2 = tab_y[i];
			}
		}
	}
	*nb_pts -= j;

	if (pente_est > pente + Donnees->Confiance * erreur_pente || pente_est < pente - Donnees->Confiance * erreur_pente)
	{
		return (0);
	}
	else
	{
		/* calcul des points */
		if (pente_est > -0.005 && pente_est < 0.005)
		{
			Zone_detect->X[0] = pente_est * Zone_detect->V[0] + ordonnee;
			Zone_detect->X[1] = Zone_detect->X[0];
		}
		else
		{
			Zone_detect->X[0] = pente_est * Zone_detect->V[0] + ordonnee;
			Zone_detect->X[1] = pente_est * Zone_detect->V[1] + ordonnee;
		}

#ifdef _WIN32_LDWS_DEBUG_
		if (Donnees->Debug)
		{
		Affiche_Median((int)Zone_detect->X[0], Zone_detect->V[0],
					   (int)Zone_detect->X[1], Zone_detect->V[1], Donnees->Aff_Median);
		}
#endif
		/* calcul des variances R */
		denominateur = (Y2 - Y1);

#ifdef _NE10_

		temp = -1 / denominateur;
		temp1 = -Y2 * temp;
		temp2 = Y1 * temp;

		createColumnMajorMatrix2x2 (&Jacob_neon, temp, temp1, -temp, temp2);
		createColumnMajorMatrix2x2 (&Jacob_T_neon, temp, -temp, temp1, temp2);

		temp  = Donnees->Conf_Pts * Donnees->Conf_Pts;
		createColumnMajorMatrix2x2 (&Cov_pts_neon, temp, 0, 0, temp);

		ne10_mulmat_2x2f_neon(&Temp_neon, &Cov_pts_neon, &Jacob_T_neon, 1);
		ne10_mulmat_2x2f_neon(&Cov_est_neon, &Jacob_neon, &Temp_neon, 1);

		Zone_detect->CX[0] =
				Zone_detect->V[0] * Zone_detect->V[0] * Cov_est_neon.c1.r1 + Cov_est_neon.c2.r2 + (Zone_detect->V[0] << 1) * Cov_est_neon.c2.r1;
		Zone_detect->CX[1] =
			Zone_detect->V[1] * Zone_detect->V[1] * Cov_est_neon.c1.r1 + Cov_est_neon.c2.r2 + (Zone_detect->V[1] << 1) * Cov_est_neon.c2.r1;
		Zone_detect->CX[2] =
			Zone_detect->V[0] * Zone_detect->V[1] * Cov_est_neon.c1.r1 +
			Cov_est_neon.c2.r2 + (Zone_detect->V[0] + Zone_detect->V[1]) * Cov_est_neon.c2.r1;

#else

		Jacob[0]	= -1 / denominateur;
		Jacob[1]	= -Jacob[0];
		Jacob[2]	= Y2 / denominateur;
		Jacob[3]	= -Y1 / denominateur;
		Cov_pts[0]	= Cov_pts[3] = Donnees->Conf_Pts * Donnees->Conf_Pts;
		Cov_pts[1]	= Cov_pts[2] = 0;

		TransposeA(Jacob, 2, 2, Jacob_T);
		ProduitAB(Cov_pts, Jacob_T, 2, 2, 2, Temp);
		ProduitAB(Jacob, Temp, 2, 2, 2, Cov_est);

		Zone_detect->CX[0] =
			Zone_detect->V[0] * Zone_detect->V[0] * Cov_est[0] + Cov_est[3] + (Zone_detect->V[0] << 1) * Cov_est[1];
		Zone_detect->CX[1] =
			Zone_detect->V[1] * Zone_detect->V[1] * Cov_est[0] + Cov_est[3] + (Zone_detect->V[1] << 1) * Cov_est[1];
		Zone_detect->CX[2] =
			Zone_detect->V[0] * Zone_detect->V[1] * Cov_est[0] +
			Cov_est[3] + (Zone_detect->V[0] + Zone_detect->V[1]) * Cov_est[1];
#endif

		if (Zone_detect->CX[0] < Donnees->Var_Min)
		{
			Zone_detect->CX[0] = Donnees->Var_Min;
		}
		if (Zone_detect->CX[1] < Donnees->Var_Min)
		{
			Zone_detect->CX[1] = Donnees->Var_Min;
		}

		return (1);
	}
}
