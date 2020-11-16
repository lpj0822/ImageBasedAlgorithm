#include <math.h>
#include "data.h"
#include "Median.h"
#include "Points.h"
#include "LDWS_Interface.h"

#ifdef _WIN32_LDWS_DEBUG_
	#include "LDWS_Debug.h"
#endif


/*
Function process:
	+ Detect the points in Zone_detect and update Zone_detect->X and Zone_detect->CX
	Fan-in : 
	        + Recherche()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int Detection_Zone(Zone *Zone_detect, Fichier *Donnees, const unsigned char *Tab_Image)
{
	int ug_sup, ud_sup, ug_inf, ud_inf;	
	int v_sup, v_inf;		
	int nb_pts = 0;				/* Num of  points detectes */
    int tab_x[LDWS_MAX_ROI_HEIGHT];				
    int tab_y[LDWS_MAX_ROI_HEIGHT];					
	double diff;


	v_sup	= Zone_detect->V[0];
	diff	= sqrt(Zone_detect->CX[0]);
	if (diff > Donnees->Largeur_Zone_Max)
	{
		ug_sup = (int)(Zone_detect->X[0] - Donnees->Largeur_Zone_Max);
		ud_sup = (int)(Zone_detect->X[0] + Donnees->Largeur_Zone_Max);
	}
	else if (diff > Donnees->Largeur_Zone_Min)
	{
		ug_sup = (int)(Zone_detect->X[0] - diff);
		ud_sup = (int)(Zone_detect->X[0] + diff);
	}
	else
	{
		ug_sup = (int)(Zone_detect->X[0] - Donnees->Largeur_Zone_Min);
		ud_sup = (int)(Zone_detect->X[0] + Donnees->Largeur_Zone_Min);
	}


	if (Zone_detect->CX[1] == Donnees->Var_Max * Donnees->Var_Max)
	{
		Zone_detect->CX[1] = Zone_detect->CX[0];
	}

	v_inf	= Zone_detect->V[1];
	diff	= sqrt(Zone_detect->CX[1]);
	if (diff > Donnees->Largeur_Zone_Max)
	{
		ug_inf = (int)(Zone_detect->X[1] - Donnees->Largeur_Zone_Max);
		ud_inf = (int)(Zone_detect->X[1] + Donnees->Largeur_Zone_Max);
	}
	else if (diff > Donnees->Largeur_Zone_Min)
	{
		ug_inf = (int)(Zone_detect->X[1] - diff);
		ud_inf = (int)(Zone_detect->X[1] + diff);
	}
	else
	{
		ug_inf = (int)(Zone_detect->X[1] - Donnees->Largeur_Zone_Min);
		ud_inf = (int)(Zone_detect->X[1] + Donnees->Largeur_Zone_Min);
	}

#ifdef _WIN32_LDWS_DEBUG_
	if (Donnees->Aff_Zone)
	{
	   Affiche_Zones(ug_sup, v_sup, ud_inf, v_inf, 1);
	   Affiche_Show(1);
	}
#endif

	if ((ud_sup < (Donnees->Marge_Image + (Donnees->Largeur_Zone_Min << 1))
		 && ud_inf <(Donnees->Marge_Image + (Donnees->Largeur_Zone_Min << 1)))
		|| (ug_sup > (Donnees->Tx - Donnees->Marge_Image - (Donnees->Largeur_Zone_Min << 1))
			&& ug_inf > (Donnees->Tx - Donnees->Marge_Image - (Donnees->Largeur_Zone_Min << 1))))
	{
        /* no usefull points */
	}
	else
	{
		if (ud_sup > Donnees->Tx - Donnees->Marge_Image)
		{
			ud_sup = Donnees->Tx - Donnees->Marge_Image;
		}
		if (ud_inf > Donnees->Tx - Donnees->Marge_Image)
		{
			ud_inf = Donnees->Tx - Donnees->Marge_Image;
		}
		if (ug_sup < Donnees->Marge_Image)
		{
			ug_sup = Donnees->Marge_Image;
		}
		if (ug_inf < Donnees->Marge_Image)
		{
			ug_inf = Donnees->Marge_Image;
		}

		switch (Zone_detect->sampleK)
		{
			case 0:
			case 1:
				Zone_detect->sampleK = 1;
				break;
			case 2:
			case 3:
				Zone_detect->sampleK = 2;
				break;
			default:
				Zone_detect->sampleK = 3;
				break;
		}

		nb_pts = PointsCandidats(ug_sup, ud_sup, v_sup, ug_inf, ud_inf,
									v_inf, tab_x, tab_y, Zone_detect, Donnees, Tab_Image);


		if (nb_pts > Donnees->Nb_Pts_Min)
		{
			if (Median(tab_x, tab_y, &nb_pts, Zone_detect, Donnees))
			{
				return (nb_pts);
			}
		}
	}

	return (0);
}


/*
Function process:
	+ Detect the points in Zone_detect
	Fan-in : 
	        + Recherche_LR()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int Detection_Zone_LR(Zone *Zone_detect, Fichier *Donnees,const unsigned char *Tab_Image)
{
	int ug_sup, ud_sup, ug_inf, ud_inf;	/* abscisses des zones de detections */
	int v_sup, v_inf;			/* ordonnees des zones de detections */
	int nb_pts = 0;				/* nombre de points detectes */
    int tab_x[LDWS_MAX_ROI_HEIGHT];					
    int tab_y[LDWS_MAX_ROI_HEIGHT];				
	double diff;

	v_sup	= Zone_detect->V[0];
	diff	= sqrt(Zone_detect->CX[0]);
	if (diff > Donnees->Largeur_Zone_Max)
	{
		ug_sup = (int)(Zone_detect->X[0] - Donnees->Largeur_Zone_Max);
		ud_sup = (int)(Zone_detect->X[0] + Donnees->Largeur_Zone_Max);
	}
	else if (diff > Donnees->Largeur_Zone_Min)
	{
		ug_sup = (int)(Zone_detect->X[0] - diff);
		ud_sup = (int)(Zone_detect->X[0] + diff);
	}
	else
	{
		ug_sup = (int)(Zone_detect->X[0] - Donnees->Largeur_Zone_Min);
		ud_sup = (int)(Zone_detect->X[0] + Donnees->Largeur_Zone_Min);
	}

	if (Zone_detect->CX[1] == Donnees->Var_Max * Donnees->Var_Max)
	{
		Zone_detect->CX[1] = Zone_detect->CX[0];
	}

	v_inf	= Zone_detect->V[1];
	diff	= sqrt(Zone_detect->CX[1]);
	if (diff > Donnees->Largeur_Zone_Max)
	{
		ug_inf = (int)(Zone_detect->X[1] - Donnees->Largeur_Zone_Max);
		ud_inf = (int)(Zone_detect->X[1] + Donnees->Largeur_Zone_Max);
	}
	else if (diff > Donnees->Largeur_Zone_Min)
	{
		ug_inf = (int)(Zone_detect->X[1] - diff);
		ud_inf = (int)(Zone_detect->X[1] + diff);
	}
	else
	{
		ug_inf = (int)(Zone_detect->X[1] - Donnees->Largeur_Zone_Min);
		ud_inf = (int)(Zone_detect->X[1] + Donnees->Largeur_Zone_Min);
	}

#ifdef _WIN32_LDWS_DEBUG_
	if (Donnees->Aff_Zone)
	{
	   Affiche_Zones(ug_sup, v_sup, ud_inf, v_inf, 1);
	}
#endif

	/* zone dans l'image + marge ? */
	if ((ud_sup < (Donnees->Marge_Image + (Donnees->Largeur_Zone_Min << 1))
		 && ud_inf <
		 (Donnees->Marge_Image + (Donnees->Largeur_Zone_Min << 1)))
		|| (ug_sup > (Donnees->Tx - Donnees->Marge_Image - (Donnees->Largeur_Zone_Min << 1))
			&& ug_inf > (Donnees->Tx - Donnees->Marge_Image - (Donnees->Largeur_Zone_Min << 1))))
	{
        /* pas de recherche de points -- no usefull points */
	}
	else
	{
		if (ud_sup > Donnees->Tx - Donnees->Marge_Image)
		{
			ud_sup = Donnees->Tx - Donnees->Marge_Image;
		}
		if (ud_inf > Donnees->Tx - Donnees->Marge_Image)
		{
			ud_inf = Donnees->Tx - Donnees->Marge_Image;
		}
		if (ug_sup < Donnees->Marge_Image)
		{
			ug_sup = Donnees->Marge_Image;
		}
		if (ug_inf < Donnees->Marge_Image)
		{
			ug_inf = Donnees->Marge_Image;
		}

		switch (Zone_detect->sampleK)
		{
			case 0:
			case 1:
				Zone_detect->sampleK = 1;
				break;
			case 2:
			case 3:
				Zone_detect->sampleK = 2;
				break;
			default:
				Zone_detect->sampleK = 3;
				break;
		}

		nb_pts = PointsCandidats_LR(ug_sup, ud_sup, v_sup, ug_inf, ud_inf,
									v_inf, tab_x, tab_y, Zone_detect, Donnees, Tab_Image);
#ifdef _LDWS_detail_DEBUG_
		my_printf("nb_pts=%d \n",nb_pts);
#endif
		if (nb_pts > Donnees->Nb_Pts_Min)
		{
			if (Median(tab_x, tab_y, &nb_pts, Zone_detect, Donnees))
			{
				return (nb_pts);
			}
		}
	}

	return (0);
}
