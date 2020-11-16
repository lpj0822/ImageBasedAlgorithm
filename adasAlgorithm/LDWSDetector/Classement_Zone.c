#include "data.h"
#include "LDWS_Interface.h"

#define COEF_ZONE    (3)

/*
Function process:
	+ Sort the CX of M_I_init by creasing, save the ID in Tab_Indices
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int Classement_Zones(const Modele_Image *M_I_init, Fichier *Donnees, int *Tab_Indices)
{
	int i = 0;
	int j = 0;
	int indice	= 0;
	int nb_zone = 0;
	int modif;
	double var		= 0;
	double var_min	= 0;			/* variances */

	for (i = 0; i < Donnees->NB_INTERVALLES * 2; i++)
	{
		Donnees->Tab_Bool[i] = 1;

		if (!M_I_init->Zones_Testees[i])
		{
			j = COEF_ZONE - (i % (Donnees->NB_INTERVALLES));

			if (j <= 0)
			{
				Donnees->Tab_Var[i] = M_I_init->CX[i + i * LDWS_NB_ZONES];
			}
			else
			{
				Donnees->Tab_Var[i] = M_I_init->CX[i + i * LDWS_NB_ZONES] * (j + 1);
			}

		}
		else
		{
			Donnees->Tab_Var[i] = Donnees->Var_Max * Donnees->Var_Max;
		}
	}

	do
	{
		var_min = Donnees->Var_Max * Donnees->Var_Max;
		modif = 0;

		for (i = 0; i < Donnees->NB_INTERVALLES * 2; i++)
		{
			if (Donnees->Tab_Bool[i] != 0)
			{
				var = Donnees->Tab_Var[i];
				if (var < var_min)
				{
					indice	= i;
					var_min = var;
					modif	= 1;
				}
			}
		}

        if (modif == 1)
		{
			Tab_Indices[nb_zone]		= indice;
			Donnees->Tab_Bool[indice]	= 0;
			nb_zone++;
		}
	}
	while (modif && nb_zone < Donnees->NB_INTERVALLES*2);

	return (nb_zone);
}

/*
Function process:
	+ Sort the CX of M_I_init between [Donnees->NB_INTERVALLES * Star, Donnees->NB_INTERVALLES * End] by creasing, save the ID in Tab_Indices
	Fan-in : 
	        + Recherche_LR()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int Classement_Zones_LR(const Modele_Image *M_I_init, Fichier *Donnees, int *Tab_Indices, const int Star, const int End)
{
	int i = 0;
	int j = 0;
	int indice	= 0;
	int nb_zone = 0;
	int modif;
	double var		= 0;
	double var_min	= 0;

	for (i = Donnees->NB_INTERVALLES * Star; i < Donnees->NB_INTERVALLES * End; i++)
	{
		Donnees->Tab_Bool[i] = 1;
		if (!M_I_init->Zones_Testees[i])
		{
			j = COEF_ZONE - (i % (Donnees->NB_INTERVALLES));
			if (j <= 0)
			{
				Donnees->Tab_Var[i] = M_I_init->CX[i + i * LDWS_NB_ZONES];
			}
			else
			{
				Donnees->Tab_Var[i] = M_I_init->CX[i + i * LDWS_NB_ZONES] * (j + 1);
			}
		}
		else
		{
			Donnees->Tab_Var[i] = Donnees->Var_Max * Donnees->Var_Max;
		}
	}

	/*
	 * classement des zones non testees dans l'ordre croissant 
	 * ascending sort the 'non testees' zones 
	 */
	do
	{
		var_min = Donnees->Var_Max * Donnees->Var_Max;
		modif = 0;
		for (i = Donnees->NB_INTERVALLES*Star; i < Donnees->NB_INTERVALLES*End; i++)
		{
			if (Donnees->Tab_Bool[i] != 0)
			{
				var = Donnees->Tab_Var[i];
				if (var < var_min)
				{
					indice	= i;
					var_min = var;
					modif	= 1;
				}
			}
		}

        if (modif == 1)
		{
			Tab_Indices[nb_zone]		= indice;
			Donnees->Tab_Bool[indice]	= 0;
			nb_zone++;
		}
	}
	while (modif && nb_zone < Donnees->NB_INTERVALLES*2);

	return (nb_zone);
}
