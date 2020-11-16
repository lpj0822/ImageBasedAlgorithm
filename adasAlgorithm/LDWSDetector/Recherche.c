#include <math.h>

#include "data.h"
#include "LDWS_Interface.h"
#include "Init_Struct.h"
#include "Classement_Zone.h"
#include "Mise_A_Jour.h"
#include "Detection_Zone.h"
#include "Sauvegarde.h"

#ifdef _WIN32_LDWS_DEBUG_
	#include "LDWS_Debug.h"
#endif

/*
I/O:	    Name		   Type	     		       Content
					    				  
[in]	    M_I_init	   const Modele_Image*	   Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_new		   Modele_Image*	       Init Parameters of 2D modele of road (Location in 2D image).

Realized function:
    + copy M_I_init to M_I_new.
*/
static void CopieEtat(const Modele_Image *M_I_init, Modele_Image *M_I_new);

/*
I/O:	Name		          Type	     		          Content
					    						  
[in]	M_I_init		      const Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in]	M_I_new		          const Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in]	M_I_est		          const Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in]	Donnees		          const Fichier*	          Global parameters to be used in LDWS .

[out]   Returned value        int                         if there is enough residual probabilities return 1; else return 0

Realized function:
    + Decide if zone of M_I_new has enough residual probabilities
*/
static int Proba_Restante(const Modele_Image *M_I_init,const Modele_Image *M_I_new,
						  const Modele_Image *M_I_est,const Fichier *Donnees);

/*
I/O:	    Name		          Type	     		          Content
					    						  
[in/out]	Zone_detect		      Zone*	                      The detect Zone to be filled.
[in]        indice_zone           const int                   The indice of zone 0-17
[in]	    M_I_init		      const Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in]	    Donnees		          const Fichier*	          Global parameters to be used in LDWS .

[out]       Returned value        int                         1

Realized function:
    + fill the detect zone 
*/
static int Remplir_Zone(Zone *Zone_detect, const int indice_zone, const Modele_Image *M_I_init, const Fichier *Donnees);

/*
I/O:	   Name		          Type	     		          Content
					    						  
[in]       add_sous           const int                   If 0 add, 1 sub
[in]	   M_I_init		      const Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]   M_I_new		      Modele_Image*	              Estimated Parameters of 2D modele of road (Location in 2D image).
[in]	   indice_zone	      const int		              The zone index.
[in]       nb_pts             const int                   The num of detected points
[in]	   Donnees		      const Fichier*	          Global parameters to be used in LDWS .

[out]      Returned value     int                         if 1 succeed ; else return 0

Realized function:
    + calculate the Proba of lines based on the detected points
*/
static int Critere(const int add_sous, const Modele_Image *M_I_init, Modele_Image *M_I_new, 
				   const int indice_zone, const int nb_pts, const Fichier *Donnees);

/*
I/O:	   Name		          Type	     		          Content
					    						  
[in]       add_sous           const int                   If 0 add, 1 sub
[in]	   M_I_init		      const Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]   M_I_new		      Modele_Image*	              Estimated Parameters of 2D modele of road (Location in 2D image).
[in]	   indice_zone	      const int		              The zone index.
[in]       nb_pts             const int                   The num of detected points
[in]	   Donnees		      const Fichier*	          Global parameters to be used in LDWS .

[out]      Returned value     int                         if 1 succeed ; else return 0

Realized function:
    + calculate the Proba of lines based on the detected points
*/
static int Critere_LR(const int add_sous,const Modele_Image *M_I_init, Modele_Image *M_I_new, 
					  const int indice_zone,const int nb_pts,const Fichier *Donnees);

//static int Choix_Gradient(Modele_Image *M_I_new, int indice_zone, Fichier *Donnees);

/****************************************************************************************************************/

/*
Function process:
	+ copy M_I_init to M_I_new
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static void CopieEtat(const Modele_Image *M_I_init, Modele_Image *M_I_new)
{
	int i;

	for (i = 0; i < LDWS_NB_ZONES; i++)
	{
		M_I_new->Zones_Testees[i]	= M_I_init->Zones_Testees[i];
		M_I_new->Zones_Detectees[i] = M_I_init->Zones_Detectees[i];
		M_I_new->templet[i]			= M_I_init->templet[i];
	}
	for (i = 0; i < (LDWS_NB_ZONES << 1); i++)
	{
		M_I_new->Xi[i]	= M_I_init->Xi[i];
		M_I_new->CXi[i] = M_I_init->CXi[i];
	}
    for (i = 0; i < LDWS_NB_BANDES; i++)
    {
        M_I_new->Proba_Bord[i]	= M_I_init->Proba_Bord[i];
        M_I_new->Grad_Bord[i]	= M_I_init->Grad_Bord[i];
		M_I_new->Proba_Bord_half[i]	= M_I_init->Proba_Bord_half[i];
    }

	M_I_new->Route	= M_I_init->Route;
	M_I_new->RouteL	= M_I_init->RouteL;
	M_I_new->RouteR	= M_I_init->RouteR;

	M_I_new->Nb_Pts = M_I_init->Nb_Pts;
	M_I_new->Nb_Pts_L = M_I_init->Nb_Pts_L;
	M_I_new->Nb_Pts_R = M_I_init->Nb_Pts_R;

	M_I_new->Proba	= M_I_init->Proba;
	M_I_new->Proba_L	= M_I_init->Proba_L;
	M_I_new->Proba_R	= M_I_init->Proba_R;
}

/*
Function process:
	+ Decide if zone of M_I_new has enough residual probabilities
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int Proba_Restante(const Modele_Image *M_I_init, const Modele_Image *M_I_new, 
						  const Modele_Image *M_I_est, const Fichier *Donnees)
{
	int i, j;
	double proba_rest = 0;

	for (j = 0; j < 2; j++)
	{
		for (i = 0; i < Donnees->NB_INTERVALLES - 1; i++)
		{
			if (!M_I_new->Zones_Testees[i + j * (Donnees->NB_INTERVALLES - 1)])
			{
				proba_rest += (double)(M_I_est->V[i] - M_I_est->V[i + 1])
							/ (double)(2 * (Donnees->V_haut - Donnees->V_bas));
			}
		}
	}

	if ((proba_rest >= M_I_est->Proba - M_I_new->Proba
		 && Donnees->Compteur < Donnees->Nb_Iterations * Donnees->Suivi && !M_I_est->Route)
		|| (proba_rest >= M_I_est->Proba - M_I_new->Proba
			&& M_I_init->Route && Donnees->Compteur < ((Donnees->Nb_Iterations * Donnees->Suivi) + 15)))
	{
		return (1);
	}

	return (0);
}

/*
Function process:
	+ fill the detect zone 
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int Remplir_Zone(Zone *Zone_detect, const int indice_zone, const Modele_Image *M_I_init, const Fichier *Donnees)
{
	int i;

	/* make sure that the zone index isn't the last zone */
	for (i = 0; i < LDWS_NB_BANDES; i++)
	{
		if (indice_zone == (i + 1) * Donnees->NB_INTERVALLES - 1)
		{
			return (0);
		}
	}

	/*
	 * remplissage de la structure Zone_detect 
	 */
	for (i = 0; i < 2; i++)
	{
		Zone_detect->V[i]	= M_I_init->V[indice_zone + i];
		Zone_detect->X[i]	= M_I_init->X[indice_zone + i];
		Zone_detect->CX[i]	= M_I_init->CX[(indice_zone + i) * (1 + LDWS_NB_ZONES)];
	}

	Zone_detect->CX[2]			= M_I_init->CX[(indice_zone + 1) + LDWS_NB_ZONES * indice_zone];
	Zone_detect->sampleK		= indice_zone % Donnees->NB_INTERVALLES;
	Zone_detect->measure.iGrad	= (int)(Donnees->templet[indice_zone].iGrad * 0.6 + 0.5);
	Zone_detect->measure.fYAve	= Donnees->templet[indice_zone].fYAve * 0.6;
	Zone_detect->measure.fAngle = 0.0;
	Zone_detect->RFlag = indice_zone/Donnees->NB_INTERVALLES;

	return (1);
}

/*
Function process:
	+ calculate the Proba of lines based on the detected points
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int Critere(const int add_sous, const Modele_Image *M_I_init, Modele_Image *M_I_new, 
				   const int indice_zone, const int nb_pts, const Fichier *Donnees)
{
	int i;
	double proba, proba_half;

	if (add_sous == 0)
	{
		M_I_new->Nb_Pts += nb_pts;
		proba			= nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 1] - M_I_init->V[0]);
		proba_half      = nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 4] - M_I_init->V[0]);
		M_I_new->Proba	+= (proba / 2);
		for (i = 0; i < 2; i++)
		{
			if (indice_zone < (i + 1) * Donnees->NB_INTERVALLES)
			{
				if((indice_zone > 2 + i*Donnees->NB_INTERVALLES) && (indice_zone < ((i+1)*Donnees->NB_INTERVALLES-1)))
				{
					M_I_new->Proba_Bord_half[i] += proba_half;
				}
				M_I_new->Proba_Bord[i] += proba;
				M_I_new->Grad_Bord[i]	= Donnees->Grad;
				return (1);
			}
		}
	}
	else
	{
		M_I_new->Nb_Pts -= nb_pts;
		proba			= nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 1] - M_I_init->V[0]);
		proba_half      = nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 4] - M_I_init->V[0]);
		M_I_new->Proba	-= (proba / 2);
		for (i = 0; i < 2; i++)
		{
			if (indice_zone <= (i + 1) * Donnees->NB_INTERVALLES)
			{
				if((indice_zone > 2 + i*Donnees->NB_INTERVALLES) && (indice_zone < ((i+1)*Donnees->NB_INTERVALLES-1)))
				{
					M_I_new->Proba_Bord_half[i] -= proba_half;
				}
				M_I_new->Proba_Bord[i] -= proba;
				if (fabs(M_I_new->Proba_Bord[i]) <= Donnees->Tolerance)
				{
					M_I_new->Proba_Bord[i]	= 0;
					M_I_new->Grad_Bord[i]	= 0;
				}

				return (1);
			}
		}
	}
	return (0);
}

/*
Function process:
	+ calculate the Proba of lines based on the detected points
	Fan-in : 
	        + Recherche_LR()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int Critere_LR(const int add_sous,const Modele_Image *M_I_init, Modele_Image *M_I_new, 
					  const int indice_zone,const int nb_pts,const Fichier *Donnees)
{
	//int i;
	double proba;

	if (add_sous == 0)
	{
		M_I_new->Nb_Pts_L += nb_pts;
		proba			= nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 1] - M_I_init->V[0]);
		M_I_new->Proba_L	+= proba;

		M_I_new->Proba_Bord[2] += proba;
		M_I_new->Grad_Bord[2]	= Donnees->Grad;
		return (1);
	}
	else
	{
		M_I_new->Nb_Pts_R += nb_pts;
		proba			= nb_pts / (double)(M_I_init->V[Donnees->NB_INTERVALLES - 1] - M_I_init->V[0]);
		M_I_new->Proba_R	+= proba;
		M_I_new->Proba_Bord[3] += proba;
		M_I_new->Grad_Bord[3]	= Donnees->Grad;
		return (1);
	}
	return (0);
}
#if 0
/*
 * Select the horizontal gradient detector 
 */
static int Choix_Gradient(Modele_Image *M_I_new, int indice_zone, Fichier *Donnees)
{
	int i;

	/*
	 * on regarde si l'on detecte une voie ou deux voies 
	 */
	for (i = 0; i < LDWS_NB_BANDES; i++)
	{
		if (indice_zone <= (i + 1) * Donnees->NB_INTERVALLES)
		{
			Donnees->Grad = M_I_new->Grad_Bord[i];
			if (Donnees->MARQUEE)
			{
				Donnees->Grad = 1;	/* configuration pour routes marquees */
			}
			return (1);
		}
	}
	return (0);
}
#endif

/****************************************************************************************************************/

/*
Function process:
	+ Search the lines of LDWS
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + CopieEtat()
			+ Classement_Zones()
			+ Proba_Restante()
			+ Remplir_Zone()
			+ Detection_Zone()
			+ M_A_J_Zone()
			+ Critere()
			+ Sauvegarde()

	ATTENTION: __________
*/
int Recherche(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees, const unsigned char *Tab_Image)
{
	int i, j;
	int num_zone = 0;			/* Num of zone */
	int indice_zone = 0;
	int nb_indice = 0;			/* Num of zones to searched */
    int Indice_Zones[LDWS_NB_ZONES + 1];/* sorted zones */
	int nb_pts = 0;
    Zone Zone_detect;			/* structure of Vi Xi CXi for each detect zon */
    Modele_Image M_I_new;		/* nouveau modele donne en entree pour les branches infrieur */

    /* copy M_I_init to M_I_new */
	CopieEtat(M_I_init, &M_I_new);

	Donnees->Arbre++;			/* tree deep */

	/* The regional classification and sort */
	nb_indice = Classement_Zones(M_I_init, Donnees, Indice_Zones);

	if (nb_indice != 0)
	{
		for (num_zone = 0; num_zone < nb_indice; num_zone++)
		{

			if (Proba_Restante(M_I_init, &M_I_new, M_I_est, Donnees))
			{
				indice_zone							= Indice_Zones[num_zone];
				M_I_new.Zones_Testees[indice_zone]	= 1;

				/*
				 * detection des points et segments dans les zones de recherche 
				 * Detection in the specified ROI of point and line segment 
				 */
				if (Remplir_Zone(&Zone_detect, indice_zone, M_I_init, Donnees))
                {
					Donnees->Compteur++;
					Donnees->Compt_indice[Donnees->Arbre]++;

					/* exit and don't progress left */
					for (i = 1; i <= Donnees->Arbre; i++)
					{
						if (Donnees->Compt_indice[i] >= (20 - Donnees->Compt_indice[1]))
						{
							for (j = i; j < LDWS_NB_ZONES; j++)
							{
								Donnees->Compt_indice[j] = 0;
							}
							num_zone = nb_indice;
						}
					}
					
                    /* Choice Detector */
                    //Choix_Gradient(M_I_new, indice_zone, Donnees);

					Zone_detect.indice_zone = indice_zone;
					nb_pts = Detection_Zone(&Zone_detect, Donnees, Tab_Image);

					if (nb_pts != 0)
					{
						M_I_new.templet[indice_zone] = Zone_detect.measure;

						M_I_new.Zones_Detectees[indice_zone] = 1;

						M_A_J_Zone(M_I_init, &M_I_new, &Zone_detect, Donnees, indice_zone);

						// wang add 20170808
						for (i = 0; i < Donnees->NB_INTERVALLES; ++i)
						{
							if (M_I_new.X[i] >= M_I_new.X[i + Donnees->NB_INTERVALLES]
								- Donnees->kWidth * 0.15 * (M_I_est->V[i] - Donnees->Cy))
							{
								//my_printf("Two lines crossing in Recherche! \n");
								nb_pts = 0;
								break;
							}
						}
					}
					if (nb_pts != 0)
					{
                        /* cacul des criteres sur la reconnaissance de la route */
						Critere(0, M_I_init, &M_I_new, indice_zone, nb_pts, Donnees);


						Sauvegarde(&M_I_new, M_I_est, Donnees);

						if (Donnees->Arbre == 1)
						{
							for (i = 0; i < LDWS_NB_ZONES; i++)
							{
								M_I_new.Zones_Testees[i]	= 0;
								M_I_new.Zones_Detectees[i]	= 0;
							}
							M_I_new.Zones_Testees[indice_zone]		= 1;
							M_I_new.Zones_Detectees[indice_zone]	= 1;
						}
#if 1
                        /* on descend dans l'arbre */
						Recherche(&M_I_new, M_I_est, Donnees, Tab_Image);

						M_I_new.Zones_Detectees[indice_zone] = 0;

                        /* remise des valeurs pour la branche inferieur */
						Critere(1, M_I_init, &M_I_new, indice_zone, nb_pts, Donnees);
#endif
					}
				}
			}
			else
			{
				Donnees->Arbre--;
				return (1);
			}
		}
	}
	Donnees->Arbre--;

	return (1);
}

/*
Function process:
	+ Search the Left and Right lines of LDWS
	Fan-in : 
	        + LDWS_RoadTracker()
	Fan-out:
	        + CopieEtat()
			+ Classement_Zones()
			+ Proba_Restante()
			+ Remplir_Zone()
			+ Detection_Zone()
			+ M_A_J_Zone()
			+ Critere()
			+ Sauvegarde()

	ATTENTION: __________
*/
int Recherche_LR(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees, const unsigned char *Tab_Image)
{
	int i;
	//int i, j;
	int num_zone = 0;			/* numero de la zone classee */
	int indice_zone = 0;
	int nb_indice = 0;			/* nombre de zones de recherche classees */
    int Indice_Zones[LDWS_NB_ZONES + 1];			/* tableau des zones class?par ordre croissantes + nb de zones classees */
	int nb_pts = 0;
    Zone Zone_detect;			/* structure Vi Xi CXi pour chercher la zone de recherche */
    Modele_Image M_I_new;		/* nouveau modele donne en entree pour les branches infrieur */

    /* copie de la branche antrieur */
	CopieEtat(M_I_init, &M_I_new);
	/*
	 * The regional classification and sort,Left 
	 */
	nb_indice = Classement_Zones_LR(M_I_init, Donnees, Indice_Zones, 2, 3);

	//Donnees->Arbre++;			/* tree */

	if (nb_indice != 0)
	{
		for (num_zone = 0; num_zone < nb_indice; num_zone++)
		{	
		    indice_zone	= Indice_Zones[num_zone];
			M_I_new.Zones_Testees[indice_zone]	= 1;
			/*
				* detection des points et segments dans les zones de recherche 
				* Detection in the specified ROI of point and line segment 
				*/
			if (Remplir_Zone(&Zone_detect, indice_zone, M_I_init, Donnees))
            {
				
				nb_pts = Detection_Zone_LR(&Zone_detect, Donnees, Tab_Image);
				if (nb_pts != 0)
				{					
					Critere_LR(0, M_I_init, &M_I_new, indice_zone, nb_pts, Donnees);

					if( M_I_new.Proba_L > Donnees->Proba_Bord)
					{
						M_I_new.RouteL=1;
						break;
					}
				}
			}
		}
	}

	/*
	 * The regional classification and sort,Righr 
	 */
	nb_indice = Classement_Zones_LR(M_I_init, Donnees, Indice_Zones, 3, 4);

	//Donnees->Arbre++;			/* tree */

	if (nb_indice != 0)
	{
		for (num_zone = 0; num_zone < nb_indice; num_zone++)
		{	
		    indice_zone	= Indice_Zones[num_zone];
			M_I_new.Zones_Testees[indice_zone]	= 1;
			/*
				* detection des points et segments dans les zones de recherche 
				* Detection in the specified ROI of point and line segment 
				*/
			if (Remplir_Zone(&Zone_detect, indice_zone, M_I_init, Donnees))
            {
				
				nb_pts = Detection_Zone_LR(&Zone_detect, Donnees, Tab_Image);
				if (nb_pts != 0)
				{					
					Critere_LR(1, M_I_init, &M_I_new, indice_zone, nb_pts, Donnees);

					if( M_I_new.Proba_R > Donnees->Proba_Bord)
					{
						M_I_new.RouteR=1;
						break;
					}
				}
			}
		}
	}	
	M_I_est->Nb_Pts_L = M_I_new.Nb_Pts_L;
	M_I_est->Proba_L  = M_I_new.Proba_L;
	M_I_est->Nb_Pts_R = M_I_new.Nb_Pts_R;
	M_I_est->Proba_R  = M_I_new.Proba_R;
	M_I_est->RouteL = M_I_new.RouteL;
	M_I_est->RouteR = M_I_new.RouteR;
	for (i = 2; i < LDWS_NB_BANDES; i++)
    {
		M_I_est->Proba_Bord[i] = M_I_new.Proba_Bord[i];
		M_I_est->Grad_Bord[i]	= M_I_new.Grad_Bord[i];
	}
	return (1);
}
