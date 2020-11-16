#include "data.h"
#include "LDWS_Interface.h"
#include "Matrice.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in]    	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Init the values of M_I_init->V and M_I_est->V
*/
static void Calcul_Ordonnees(Modele_Image *M_I_init, Modele_Image *M_I_est, const Fichier *Donnees);

static void Jacobien_Une_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees);

static void Jacobien_Deux_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees);

static void Jacobien_Trip_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees);

static void Jacobien_Steer_Angle(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees);

/***********************************************************************************************************************/

/*
Function process:
	+ Calculation of the initial y coordinate of search area,given the value of M_I_init->V and M_I_est->V
	Fan-in : 
	        + Initialisation()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static void Calcul_Ordonnees(Modele_Image *M_I_init, Modele_Image *M_I_est, const Fichier *Donnees)
{
	int i, j, v;
	double intervalle;
	int somme = 0;

	/*
	 * calcul des ordonnees de ces zones de recherche 
	 */
	v = Donnees->V_haut;
	for (i = 0; i < Donnees->NB_INTERVALLES - 1; i++)
	{
		somme += i;
	}

	intervalle = (Donnees->V_bas - v - (Donnees->NB_INTERVALLES - 1) * Donnees->H_zone) / (double)somme;

	for (i = 0; i < Donnees->NB_INTERVALLES; i++)
	{
		for (j = 0; j < LDWS_NB_BANDES; j++)
		{
			M_I_init->V[i + j * Donnees->NB_INTERVALLES] = v;
		}
		v += (int)(Donnees->H_zone + i * intervalle);
	}

	for (i = 0; i < LDWS_NB_ZONES; i++)
	{
		M_I_est->V[i] = M_I_init->V[i];
	}
}

/*
 * The Jacobi matrix and the calculation method of vector 
 */
static void Jacobien_Une_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees)
{
	int i, j, d, g, v;
	double v_evalpha = 0, div = 0, temp;

	/*
	 * calculs des vecteurs moyens et du jacobien Xd ->Jp 
	 */
	for (g = 0; g < Donnees->NB_INTERVALLES; g++)
	{
		d			= g + Donnees->NB_INTERVALLES;
		v			= Donnees->Cy - M_I_init->V[g];
		v_evalpha	= v - (double)(Donnees->Ev * M_3D_init->Param[3]);	/* Vi - Ev * alpha */
		div			= v_evalpha / (double)(-Donnees->Ev * Donnees->Z0);	/* (Vi - Ev * alpha) / (Ev * z0) */
		temp		= Donnees->Eu * (-div * M_3D_init->Param[1] + M_3D_init->Param[2]
					+ M_3D_init->Param[4] / (2 * div) + M_3D_init->Param[5] / (6 * div * div)) + Donnees->Cx;
		M_3D_init->X[g] = M_I_init->X[g] = temp;	/* */
		M_3D_init->X[d] = M_I_init->X[d] = temp + div * M_3D_init->Param[0] * Donnees->Eu;	/* */

		temp = Donnees->Eu * div;
		Jacobien[g * LDWS_NB_PARAM + 0] = 0;//d(I_x1_l)/d(w)
		Jacobien[d * LDWS_NB_PARAM + 0] = temp;//d(I_x1_r)/d(w)
		Jacobien[g * LDWS_NB_PARAM + 1] = -temp;//d(I_x1_l)/d(l)
		Jacobien[d * LDWS_NB_PARAM + 1] = -temp;//d(I_x1_r)/d(l)
		Jacobien[g * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(I_x1_l)/d(fai)
		Jacobien[d * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(I_x1_r)/d(fai)

		temp = Donnees->Eu * (- M_3D_init->Param[1] / Donnees->Z0
			 + (M_3D_init->Param[4] / 2 - (M_3D_init->Param[5] / (3 * div))) / (Donnees->Z0 * div * div));
		Jacobien[g * LDWS_NB_PARAM + 3] = temp;//d(I_x1_l)/d(theta)
		Jacobien[d * LDWS_NB_PARAM + 3] = temp + Donnees->Eu * M_3D_init->Param[0] / Donnees->Z0;//d(I_x1_r)/d(theta)

		temp = Donnees->Eu / (2.0 * div);
		Jacobien[g * LDWS_NB_PARAM + 4] = temp;//d(I_x1_l)/d(c_h0)
		Jacobien[d * LDWS_NB_PARAM + 4] = temp;//d(I_x1_r)/d(c_h0)

		temp /= 3.0 * div;
		Jacobien[g * LDWS_NB_PARAM + 5] = temp;//d(I_x1_l)/d(c_h1)
		Jacobien[d * LDWS_NB_PARAM + 5] = temp;//d(I_x1_r)/d(c_h1)
	}

	/*
	 * fin de remplissage du jacobien et de X param Xl->Jp 
	 */
	for (i = LDWS_NB_ZONES; i < Donnees->NB_Z_P; i++)
	{
		v = i * LDWS_NB_PARAM;
		g = i - LDWS_NB_ZONES;
		for (j = 0; j < LDWS_NB_PARAM; j++)
		{
			Jacobien[v + j] = 0;
		}

		Jacobien[v + g] = 1;
		M_3D_init->X[i] = M_3D_init->Param[g];
	}
}

static void Jacobien_Deux_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees)
{
	int i, j, d, c, g, v;
	double v_evalpha = 0, div = 0;

	/*
	 * calculs des vecteurs moyens et du jacobien 
	 */
	for (g = 0; g < Donnees->NB_INTERVALLES; g++)
	{
		c			= g + Donnees->NB_INTERVALLES;
		d			= g + 2 * Donnees->NB_INTERVALLES;
		v			= Donnees->Cy - M_I_init->V[g];
		v_evalpha	= v - (double)(Donnees->Ev * M_3D_init->Param[3]);
		div			= v_evalpha / (double)(Donnees->Ev * Donnees->Z0);

		M_3D_init->X[d] = M_I_init->X[d] = Donnees->Eu * (-div * (M_3D_init->Param[1] + M_3D_init->Param[0])
						+ M_3D_init->Param[2] + M_3D_init->Param[4] / (2 * div)) + Donnees->Cx;
		M_3D_init->X[c] = M_I_init->X[c] = Donnees->Eu * (-div * M_3D_init->Param[1] 
						+ M_3D_init->Param[2] + M_3D_init->Param[4] / (2 * div)) + Donnees->Cx;
		M_3D_init->X[g] = M_I_init->X[g] = Donnees->Eu * (-div * (M_3D_init->Param[1] - M_3D_init->Param[0])
						+ M_3D_init->Param[2] + M_3D_init->Param[4] / (2 * div)) + Donnees->Cx;

		/*
		 * jacobien 
		 */
		Jacobien[d * LDWS_NB_PARAM + 0] = -Donnees->Eu * div;
		Jacobien[c * LDWS_NB_PARAM + 0] = 0;
		Jacobien[g * LDWS_NB_PARAM + 0] = Donnees->Eu * div;
		Jacobien[d * LDWS_NB_PARAM + 1] = -Donnees->Eu * div;
		Jacobien[c * LDWS_NB_PARAM + 1] = -Donnees->Eu * div;
		Jacobien[g * LDWS_NB_PARAM + 1] = -Donnees->Eu * div;
		Jacobien[d * LDWS_NB_PARAM + 2] = Donnees->Eu;
		Jacobien[c * LDWS_NB_PARAM + 2] = Donnees->Eu;
		Jacobien[g * LDWS_NB_PARAM + 2] = Donnees->Eu;
		Jacobien[g * LDWS_NB_PARAM + 3] = Donnees->Eu * ((M_3D_init->Param[1] + M_3D_init->Param[0]) / Donnees->Z0
										+ (Donnees->Ev * Donnees->Ev * Donnees->Z0
										* M_3D_init->Param[4]) / (2 * (v_evalpha) * (v_evalpha)));
		Jacobien[d * LDWS_NB_PARAM + 3] = Donnees->Eu * (M_3D_init->Param[1] / Donnees->Z0
										+ (Donnees->Ev * Donnees->Ev * Donnees->Z0
										* M_3D_init->Param[4]) / (2 * (v_evalpha) * (v_evalpha)));
		Jacobien[g * LDWS_NB_PARAM + 3] = Donnees->Eu * ((M_3D_init->Param[1] - M_3D_init->Param[0]) / Donnees->Z0
										+ (Donnees->Ev * Donnees->Ev * Donnees->Z0
										* M_3D_init->Param[4]) / (2 * (v_evalpha) * (v_evalpha)));
		Jacobien[d * LDWS_NB_PARAM + 4] = Donnees->Eu / (2 * div);
		Jacobien[c * LDWS_NB_PARAM + 4] = Donnees->Eu / (2 * div);
		Jacobien[g * LDWS_NB_PARAM + 4] = Donnees->Eu / (2 * div);
		Jacobien[d * LDWS_NB_PARAM + 5] = 0;
		Jacobien[c * LDWS_NB_PARAM + 5] = 0;
		Jacobien[g * LDWS_NB_PARAM + 5] = 0;
	}

	/*
	 * fin de remplissage du jacobien et de X param 
	 */
	for (i = LDWS_NB_ZONES; i < Donnees->NB_Z_P; i++)
	{
		for (j = 0; j < LDWS_NB_PARAM; j++)
		{
			Jacobien[i * LDWS_NB_PARAM + j] = 0;
		}
		Jacobien[i * LDWS_NB_PARAM + (i - LDWS_NB_ZONES)]	= 1;
		M_3D_init->X[i]										= M_3D_init->Param[i - LDWS_NB_ZONES];
	}
}

/*
Function process:
	+ Calculation of the initial X coordinate of M_I_init->X and M_3D_init->X, Calculation the Jacobien
	Fan-in : 
	        + Initialisation()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static void Jacobien_Trip_Voie(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees)
{
	int i, j, d, g, c, b, v;
	double v_evalpha = 0, div = 0, temp;

	/*
	 * calculs des vecteurs moyens et du jacobien Xd ->Jp 
	 */
	for (g = 0; g < Donnees->NB_INTERVALLES; g++)
	{
		d			= g + Donnees->NB_INTERVALLES;
		c			= g + 2*Donnees->NB_INTERVALLES;
		b			= g + 3*Donnees->NB_INTERVALLES;

		v			= Donnees->Cy - M_I_init->V[g];
		v_evalpha	= v - (double)(Donnees->Ev * M_3D_init->Param[3]);	/* Vi - Ev * alpha */
		div			= v_evalpha / (double)(- Donnees->Ev * Donnees->Z0);	/* (Vi - Ev * alpha) / (Ev * z0) */
		temp		= Donnees->Eu * (-div * M_3D_init->Param[1] + M_3D_init->Param[2]
					+ M_3D_init->Param[4] / (2 * div) + M_3D_init->Param[5] / (6 * div * div)) + Donnees->Cx;

		M_3D_init->X[g] = M_I_init->X[g] = temp;	/* */
		M_3D_init->X[d] = M_I_init->X[d] = temp + div * M_3D_init->Param[0] * Donnees->Eu;	/* */
		M_3D_init->X[c] = M_I_init->X[c] = temp - div * M_3D_init->Param[0] * Donnees->Eu;
		M_3D_init->X[b] = M_I_init->X[b] = temp + 2*div * M_3D_init->Param[0] * Donnees->Eu;
		
		temp = Donnees->Eu * div;
		Jacobien[g * LDWS_NB_PARAM + 0] = 0;//d(X[g])/d(w)
		Jacobien[d * LDWS_NB_PARAM + 0] = temp;//d(X[d])/d(w)
		Jacobien[c * LDWS_NB_PARAM + 0] = -temp;//d(X[c])/d(w)
		Jacobien[b * LDWS_NB_PARAM + 0] = 2*temp;//d(X[b])/d(w)
		

		Jacobien[g * LDWS_NB_PARAM + 1] = -temp;//d(X[g])/d(l)
		Jacobien[d * LDWS_NB_PARAM + 1] = -temp;//d(X[g])/d(l)
		Jacobien[c * LDWS_NB_PARAM + 1] = -temp;//d(X[g])/d(l)
		Jacobien[b * LDWS_NB_PARAM + 1] = -temp;//d(X[g])/d(l)

		Jacobien[g * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(X[g])/d(fai)
		Jacobien[d * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(X[g])/d(fai)
		Jacobien[c * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(X[g])/d(fai)
		Jacobien[b * LDWS_NB_PARAM + 2] = Donnees->Eu;//d(X[g])/d(fai)

		temp = Donnees->Eu * (- M_3D_init->Param[1] / Donnees->Z0
			 + (M_3D_init->Param[4] / 2 - (M_3D_init->Param[5] / (3 * div))) / (Donnees->Z0 * div * div));
		Jacobien[g * LDWS_NB_PARAM + 3] = temp;                                                    //d(X[g])/d(theta)
		Jacobien[d * LDWS_NB_PARAM + 3] = temp + Donnees->Eu * M_3D_init->Param[0] / Donnees->Z0;  //d(X[d])/d(theta)
		Jacobien[c * LDWS_NB_PARAM + 3] = temp - Donnees->Eu * M_3D_init->Param[0] / Donnees->Z0;  //d(X[c])/d(theta)
		Jacobien[b * LDWS_NB_PARAM + 3] = temp + 2*Donnees->Eu * M_3D_init->Param[0] / Donnees->Z0;//d(X[b])/d(theta)
		


		temp = Donnees->Eu / (2.0 * div);
		Jacobien[g * LDWS_NB_PARAM + 4] = temp;//d(X[g])/d(c_h0)
		Jacobien[d * LDWS_NB_PARAM + 4] = temp;//d(X[d])/d(c_h0)
		Jacobien[c * LDWS_NB_PARAM + 4] = temp;//d(X[c])/d(c_h0)
		Jacobien[b * LDWS_NB_PARAM + 4] = temp;//d(X[b])/d(c_h0)


		temp /= 3.0 * div;
		Jacobien[d * LDWS_NB_PARAM + 5] = temp;//d(X[g])/d(c_h1)
		Jacobien[g * LDWS_NB_PARAM + 5] = temp;//d(X[d])/d(c_h1)
		Jacobien[c * LDWS_NB_PARAM + 5] = temp;//d(X[c])/d(c_h1)
		Jacobien[b * LDWS_NB_PARAM + 5] = temp;//d(X[b])/d(c_h1)
	}

	/*
	 * fin de remplissage du jacobien et de X param Xl->Jp 
	 */
	for (i = LDWS_NB_ZONES; i < Donnees->NB_Z_P; i++)
	{
		v = i * LDWS_NB_PARAM;
		g = i - LDWS_NB_ZONES;
		for (j = 0; j < LDWS_NB_PARAM; j++)
		{
			Jacobien[v + j] = 0;
		}

		Jacobien[v + g] = 1;
		M_3D_init->X[i] = M_3D_init->Param[g];
	}
}

static void Jacobien_Steer_Angle(Modele_Image *M_I_init, Modele_3D *M_3D_init, double *Jacobien, const Fichier *Donnees)
{
	int i, j, d, g, v;
	double v_evalpha = 0, div = 0;

	/*
	 * calculs des vecteurs moyens et du jacobien 
	 */
	for (g = 0; g < Donnees->NB_INTERVALLES; g++)
	{
		d			= g + Donnees->NB_INTERVALLES;
		v			= Donnees->Ty / 2 - M_3D_init->V[g];
		v_evalpha	= v - (double)(Donnees->Ev * M_3D_init->Param[3]);
		div			= v_evalpha / (double)(Donnees->Ev * Donnees->Z0);

		M_3D_init->X[d] = M_I_init->X[d] = Donnees->Eu * (-div * M_3D_init->Param[1]
						+ M_3D_init->Param[2] + M_3D_init->Param[4] / (2 * div)) + Donnees->Tx / 2;
		M_3D_init->X[g] = M_I_init->X[g] = Donnees->Eu * (-div * (M_3D_init->Param[1] - M_3D_init->Param[0])
						+ M_3D_init->Param[2] + M_3D_init->Param[4] / (2 * div)) + Donnees->Tx / 2;

		/*
		 * jacobien 
		 */
		Jacobien[d * LDWS_NB_PARAM + 0] = 0;
		Jacobien[g * LDWS_NB_PARAM + 0] = Donnees->Eu * div;
		Jacobien[d * LDWS_NB_PARAM + 1] = -Donnees->Eu * div;
		Jacobien[g * LDWS_NB_PARAM + 1] = -Donnees->Eu * div;
		Jacobien[d * LDWS_NB_PARAM + 2] = Donnees->Eu;
		Jacobien[g * LDWS_NB_PARAM + 2] = Donnees->Eu;
		Jacobien[d * LDWS_NB_PARAM + 3] = Donnees->Eu * (M_3D_init->Param[1] / Donnees->Z0
										+ (Donnees->Ev * Donnees->Ev * Donnees->Z0
										* M_3D_init->Param[4]) / (2 * (v_evalpha) * (v_evalpha)));
		Jacobien[g * LDWS_NB_PARAM + 3] = Donnees->Eu * ((M_3D_init->Param[1] - M_3D_init->Param[0]) / Donnees->Z0
										+ (Donnees->Ev * Donnees->Ev * Donnees->Z0
										* M_3D_init->Param[4]) / (2 * (v_evalpha) * (v_evalpha)));
		Jacobien[d * LDWS_NB_PARAM + 4] = Donnees->Eu / (2 * div);
		Jacobien[g * LDWS_NB_PARAM + 4] = Donnees->Eu / (2 * div);
		Jacobien[d * LDWS_NB_PARAM + 5] = 0;
		Jacobien[g * LDWS_NB_PARAM + 5] = 0;
	}

	/*
	 * fin de remplissage du jacobien et de X param 
	 */
	for (i = LDWS_NB_ZONES; i < LDWS_NB_ZONES + LDWS_NB_PARAM; i++)
	{
		for (j = 0; j < LDWS_NB_PARAM; j++)
		{
			Jacobien[i * LDWS_NB_PARAM + j] = 0;
		}
		Jacobien[i * LDWS_NB_PARAM + (i - LDWS_NB_ZONES)]	= 1;
		M_3D_init->X[i]										= M_3D_init->Param[i - LDWS_NB_ZONES];
	}
}

/*
Function process:
	+ Calculation of the initial value of M_I_init, M_I_est and M_3D_init
	Fan-in : 
	        + Initialisation()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void Initialisation(Modele_Image *M_I_init, Modele_Image *M_I_est, Modele_3D *M_3D_init, const Fichier *Donnees)
{
	int i, j;

	double Jacobien[(LDWS_NB_ZONES + LDWS_NB_PARAM) * LDWS_NB_PARAM];//30*6
	double Jacobien_T[(LDWS_NB_ZONES + LDWS_NB_PARAM) * LDWS_NB_PARAM];//6*30
	double Temp[(LDWS_NB_ZONES + LDWS_NB_PARAM) * LDWS_NB_PARAM];//6*30

	Calcul_Ordonnees(M_I_init, M_I_est, Donnees);

	if (Donnees->VEHICULE == VELAC)
	{
		if (LDWS_NB_BANDES == 2)
		{
			Jacobien_Une_Voie(M_I_init, M_3D_init, Jacobien, Donnees);
		}
		if (LDWS_NB_BANDES == 3)
		{
			Jacobien_Deux_Voie(M_I_init, M_3D_init, Jacobien, Donnees);
		}
		if (LDWS_NB_BANDES == 4)
		{
			Jacobien_Trip_Voie(M_I_init, M_3D_init, Jacobien, Donnees);
		}
	}
	if (Donnees->VEHICULE == TRACTEUR)
	{
		Jacobien_Steer_Angle(M_I_init, M_3D_init, Jacobien, Donnees);
	}

	/*
	 * calcul de la matrice de covariance du modle 
	 * The calculation model of the covariance matrix 
	 */
	TransposeA(Jacobien, Donnees->NB_Z_P, LDWS_NB_PARAM, Jacobien_T);	/* Jp->JpT */
	ProduitAB(M_3D_init->C_Param, Jacobien_T, LDWS_NB_PARAM, LDWS_NB_PARAM, Donnees->NB_Z_P, Temp);	/* Cp*JpT (6*6) *(6*30)*/
	ProduitAB(Jacobien, Temp, Donnees->NB_Z_P, LDWS_NB_PARAM, Donnees->NB_Z_P, M_3D_init->CX);	/* Cx = Jp  *Cp *  JpT */

	/*
	 * copie de CX dans le modele image init 
	 */
	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		for (i = 0; i < LDWS_NB_ZONES; i++)
		{
			M_I_init->CX[j * LDWS_NB_ZONES + i] = M_3D_init->CX[j * Donnees->NB_Z_P + i];
		}
	}
}
