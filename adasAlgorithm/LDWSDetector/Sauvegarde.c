#include "data.h"
#include "LDWS_Interface.h"

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    M_I_new		          Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).

Realized function:
    + Copy M_I_new to M_I_est.
*/
static void Copie(const Modele_Image *M_I_new, Modele_Image *M_I_est);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    M_I_new		          Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + If the estimated proba of M_I_new satisfied  Donnees->Proba and Donnees->Proba_Bord return 1;else return 0.
*/
static int Proba_Ok(const Modele_Image *M_I_new, const Fichier *Donnees);

/******************************************************************************************/

/*
Function process:
	+ Copy M_I_new to M_I_est, 
	Fan-in : 
	        + Sauvegarde()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static void Copie(const Modele_Image *M_I_new, Modele_Image *M_I_est)
{
	int i, j;

	for (j = 0; j < LDWS_NB_ZONES; j++)
	{
		M_I_est->X[j]		= M_I_new->X[j];
		M_I_est->templet[j] = M_I_new->templet[j];

		M_I_est->Zones_Testees[j]	= M_I_new->Zones_Testees[j];
		M_I_est->Zones_Detectees[j] = M_I_new->Zones_Detectees[j];
		for (i = 0; i < LDWS_NB_ZONES; i++)
		{
			M_I_est->CX[j * LDWS_NB_ZONES + i] = M_I_new->CX[j * LDWS_NB_ZONES + i];
		}
	}
	for (j = 0; j < (LDWS_NB_ZONES << 1); j++)
	{
		M_I_est->Xi[j]	= M_I_new->Xi[j];
		M_I_est->CXi[j] = M_I_new->CXi[j];
	}
    for (i = 0; i < LDWS_NB_BANDES; i++)
    {
        M_I_est->Proba_Bord[i]	= M_I_new->Proba_Bord[i];
        M_I_est->Grad_Bord[i]	= M_I_new->Grad_Bord[i];
		M_I_est->Proba_Bord_half[i]	= M_I_new->Proba_Bord_half[i];
    }

	M_I_est->Nb_Pts	= M_I_new->Nb_Pts;
	M_I_est->Proba	= M_I_new->Proba;
}

/*
Function process:
	+ If the estimated proba of M_I_new satisfied  Donnees->Proba and Donnees->Proba_Bord return 1;else return 0.
	Fan-in : 
	        + Sauvegarde()
	Fan-out:
	        + N/A
*/
static int Proba_Ok(const Modele_Image *M_I_new, const Fichier *Donnees)
{
	int i, compteur = 0;

	if (M_I_new->Proba > Donnees->Proba)
	{
		for (i = 0; i < 2; i++)
		{
			if (M_I_new->Proba_Bord[i] > Donnees->Proba_Bord)
			{
				compteur++;
			}
		}
		if (compteur == 2)
		{
			return (1);
		}
	}
	return (0);
}

/******************************************************************************************/

/*
Function process:
	+ decide if M_I_new satisfied the Proba, if satisfied update M_I_est.  
	Fan-in : 
	        + main()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
void Sauvegarde(Modele_Image *M_I_new, Modele_Image *M_I_est, Fichier *Donnees)
{
	if (!M_I_est->Route)
	{
		if (M_I_new->Proba > M_I_est->Proba)
		{
			Copie(M_I_new, M_I_est);
		}

		if (Proba_Ok(M_I_new, Donnees))
		{
			Donnees->Compteur	= (Donnees->Nb_Iterations * Donnees->Suivi);
			M_I_est->Route		= 1;
			M_I_new->Route		= 1;
			Copie(M_I_new, M_I_est);
		}
	}
	else
	{
		if (Proba_Ok(M_I_new, Donnees))
		{
			Copie(M_I_new, M_I_est);
		}
	}
}
