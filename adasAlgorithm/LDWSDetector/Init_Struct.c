#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "data.h"
#include "LDWS_Interface.h"
#include "Caractere.h"

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS.

   Realized function:
   + Malloc the memory of Donnees
   */
static void Allocation_Modele_Fichier(Fichier *Donnees);

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	M_3D_new		      Modele_3D*	      Init Parameters of 3D modele of road.
   [in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.

   Realized function:
   + Malloc the memory of M_3D_new
   */
static void Allocation_Modele_3D(Modele_3D * M_3D_new, const Fichier *Donnees);

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
   [in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.

   Realized function:
   + Malloc the memory of M_Filter
   */
static void Allocation_Modele_Filter(Modele_Filter *M_Filter, const Fichier *Donnees);

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road.
   [in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.

   Realized function:
   + Init the value of M_3D_init->Param and M_3D_init->C_Param
   */
static void Modele_VELAC(Modele_3D *M_3D_init, const Fichier *Donnees);

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	L_output		      LDWS_Output*	      Output parameters model.
   [in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.

   Realized function:
   + Malloc the memory of L_output
   */
static void Allocation_Modele_OutPut(LDWS_Output *L_output, const Fichier *Donnees);

/*
   I/O:	    Name		          Type	     		  Content

   [in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.

   Realized function:
   + Free the memory of M_Filter
   */
static void Free_Modele_Filter(Modele_Filter *M_Filter);

/*
   I/O:	    Name		Type	     	  Content

   [in/out]	M_3D		Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).

   Realized function:
   + Free the memory of M_3D
   */
static void Free_Modele_3D(Modele_3D * M_3D);

/*******************************************************************************************/

/*
   Function process:
   + read the config files and Init Donnees
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + N/A
ATTENTION: __________
*/
int Get_Fichier(Fichier *Donnees,const char *fichier, const char *fichierCustom)
{
    int ret = 0;
    int customInit = 0;
    int initCenter = 0;
    int carWideth, leftDeviation, cameraHigh, rightDeviation, roadWidth,vehicleWarning, cameraPosx;
    int firstLineY, secondLineY, verticalLineX, horizontalLineY;

#if 1
    if (fichier != NULL)
    {
        ret = LectureParametresInt(fichier, "RUNFLAG=", "%d", &Donnees->runFlag);
        if(!ret)
        {
            printf("Can not find RUNFLAG= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "VEHICULE=", "%d", &Donnees->VEHICULE);
        if(!ret)
        {
            printf("Can not find VEHICULE= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "MARQUEE=", "%d", &Donnees->MARQUEE);
        if(!ret)
        {
            printf("Can not find MARQUEE= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Eu_N=", "%d", &Donnees->Eu_N);
        if(!ret)
        {
            printf("Can not find Eu_N= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Ev_N=", "%d", &Donnees->Ev_N);
        if(!ret)
        {
            printf("Can not find Ev_N= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Tx_N=", "%d", &Donnees->Tx_N);
        if(!ret)
        {
            printf("Can not find Tx_N= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Ty_N=", "%d", &Donnees->Ty_N);
        if(!ret)
        {
            printf("Can not find Ty_N= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Cx_N=", "%d", &Donnees->Cx_N);
        if(!ret)
        {
            printf("Can not find Cx_N= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Cy_N=", "%d", &Donnees->Cy_N);
        if(!ret)
        {
            printf("Can not find Cy_N= in fichier!\n");
            return ret;
        }

        if(Donnees->runFlag)
        {
            ret = LectureParametresInt(fichier, "Eu_W=", "%d", &Donnees->Eu);
            if(!ret)
            {
                printf("Can not find Eu_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "Ev_W=", "%d", &Donnees->Ev);
            if(!ret)
            {
                printf("Can not find Ev_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "Tx_W=", "%d", &Donnees->Tx);
            if(!ret)
            {
                printf("Can not find Tx_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "Ty_W=", "%d", &Donnees->Ty);
            if(!ret)
            {
                printf("Can not find Ty_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "Cx_W=", "%d", &Donnees->Cx);
            if(!ret)
            {
                printf("Can not find Cx_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "Cy_W=", "%d", &Donnees->Cy);
            if(!ret)
            {
                printf("Can not find Cy_W= in fichier!\n");
                return ret;
            }
        }
        else
        {
            Donnees->Eu = Donnees->Eu_N;
            Donnees->Ev = Donnees->Ev_N;
            Donnees->Cx = Donnees->Cx_N;
            Donnees->Cy = Donnees->Cy_N;
            Donnees->Tx = Donnees->Tx_N;
            Donnees->Ty = Donnees->Ty_N;
        }

        ret = LectureParametresdouble(fichier, "Z0=", "%lf", &Donnees->Z0);
        if(!ret)
        {
            printf("Can not find Z0= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Largeur=", "%lf", &Donnees->Largeur);
        if(!ret)
        {
            printf("Can not find Largeur= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "X0=", "%lf", &Donnees->X0);
        if(!ret)
        {
            printf("Can not find X0= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Alpha=", "%lf", &Donnees->Alpha);
        if(!ret)
        {
            printf("Can not find Alpha= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Psi=", "%lf", &Donnees->Psi);
        if(!ret)
        {
            printf("Can not find Psi= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Ch=", "%lf", &Donnees->Ch);
        if(!ret)
        {
            printf("Can not find Ch= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Cl=", "%lf", &Donnees->Cl);
        if(!ret)
        {
            printf("Can not find Cl= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_Largeur=", "%lf", &Donnees->Dev_Largeur);
        if(!ret)
        {
            printf("Can not find Dev_Largeur= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_X0=", "%lf", &Donnees->Dev_X0);
        if(!ret)
        {
            printf("Can not find Dev_X0= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_Alpha=", "%lf", &Donnees->Dev_Alpha);
        if(!ret)
        {
            printf("Can not find Dev_Alpha= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_Psi=", "%lf", &Donnees->Dev_Psi);
        if(!ret)
        {
            printf("Can not find Dev_Psi= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_Ch=", "%lf", &Donnees->Dev_Ch);
        if(!ret)
        {
            printf("Can not find Dev_Ch= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Dev_Cl=", "%lf", &Donnees->Dev_Cl);
        if(!ret)
        {
            printf("Can not find Dev_Cl= in fichier!\n");
            return ret;
        }

        if(Donnees->runFlag)
        {
            ret = LectureParametresInt(fichier, "V_haut_W=", "%d", &Donnees->V_haut);
            if(!ret)
            {
                printf("Can not find V_haut_W= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "V_bas_W=", "%d", &Donnees->V_bas);
            if(!ret)
            {
                printf("Can not find V_bas_W= in fichier!\n");
                return ret;
            }
        }
        else
        {
            ret = LectureParametresInt(fichier, "V_haut_N=", "%d", &Donnees->V_haut);
            if(!ret)
            {
                printf("Can not find V_haut_N= in fichier!\n");
                return ret;
            }

            ret = LectureParametresInt(fichier, "V_bas_N=", "%d", &Donnees->V_bas);
            if(!ret)
            {
                printf("Can not find V_bas_N= in fichier!\n");
                return ret;
            }
        }

        ret = LectureParametresInt(fichier, "H_zone=", "%d", &Donnees->H_zone);
        if(!ret)
        {
            printf("Can not find H_zone= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Var_Max=", "%d", &Donnees->Var_Max);
        if(!ret)
        {
            printf("Can not find Var_Max= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Var_Min=", "%d", &Donnees->Var_Min);
        if(!ret)
        {
            printf("Can not find Var_Min= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Largeur_Zone_Min=", "%d", &Donnees->Largeur_Zone_Min);
        if(!ret)
        {
            printf("Can not find Largeur_Zone_Min= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Largeur_Zone_Max=", "%d", &Donnees->Largeur_Zone_Max);
        if(!ret)
        {
            printf("Can not find Largeur_Zone_Max= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Marge_Image=", "%d", &Donnees->Marge_Image);
        if(!ret)
        {
            printf("Can not find Marge_Image= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Nb_Pts_Min=", "%d", &Donnees->Nb_Pts_Min);
        if(!ret)
        {
            printf("Can not find Nb_Pts_Min= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Seuil_Grad=", "%d", &Donnees->Seuil_Grad);
        if(!ret)
        {
            printf("Can not find Seuil_Grad= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Grad_Core_Length=", "%d", &Donnees->Grad_Core_Length);
        if(!ret)
        {
            printf("Can not find Grad_Core_Length= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "L_Bande_Sup=", "%d", &Donnees->L_Bande_Sup);
        if(!ret)
        {
            printf("Can not find L_Bande_Sup= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "L_Bande_Inf=", "%d", &Donnees->L_Bande_Inf);
        if(!ret)
        {
            printf("Can not find L_Bande_Inf= in fichier!\n");
            return ret;
        }


        ret = LectureParametresInt(fichier, "Nb_Tir_Med=", "%d", &Donnees->Nb_Tir_Med);
        if(!ret)
        {
            printf("Can not find Nb_Tir_Med= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Confiance=", "%lf", &Donnees->Confiance);
        if(!ret)
        {
            printf("Can not find Confiance= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Ecart_Type=", "%lf", &Donnees->Ecart_Type);
        if(!ret)
        {
            printf("Can not find Ecart_Type= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Residu_Max=", "%d", &Donnees->Residu_Max);
        if(!ret)
        {
            printf("Can not find Residu_Max= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Coef_Median=", "%lf", &Donnees->Coef_Median);
        if(!ret)
        {
            printf("Can not find Coef_Median= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Conf_Pts=", "%d", &Donnees->Conf_Pts);
        if(!ret)
        {
            printf("Can not find Conf_Pts= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Erreur_mini=", "%lf", &Donnees->Erreur_mini);
        if(!ret)
        {
            printf("Can not find Erreur_mini= in fichier!\n");
            return ret;
        }


        ret = LectureParametresInt(fichier, "Nb_Iterations=", "%d", &Donnees->Nb_Iterations);
        if(!ret)
        {
            printf("Can not find Nb_Iterations= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Proba=", "%lf", &Donnees->Proba);
        if(!ret)
        {
            printf("Can not find Proba= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Proba_Bord=", "%lf", &Donnees->Proba_Bord);
        if(!ret)
        {
            printf("Can not find Proba_Bord= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Tolerance=", "%lf", &Donnees->Tolerance);
        if(!ret)
        {
            printf("Can not find Tolerance= in fichier!\n");
            return ret;
        }


        ret = LectureParametresdouble(fichier, "Lambda_Y=", "%lf", &Donnees->Lambda_Y);
        if(!ret)
        {
            printf("Can not find Lambda_Y= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_L=", "%lf", &Donnees->Delta_L);
        if(!ret)
        {
            printf("Can not find Delta_L= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_X0=", "%lf", &Donnees->Delta_X0);
        if(!ret)
        {
            printf("Can not find Delta_X0= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_Alpha=", "%lf", &Donnees->Delta_Alpha);
        if(!ret)
        {
            printf("Can not find Delta_Alpha= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_Psi=", "%lf", &Donnees->Delta_Psi);
        if(!ret)
        {
            printf("Can not find Delta_Psi= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_Ch=", "%lf", &Donnees->Delta_Ch);
        if(!ret)
        {
            printf("Can not find Delta_Ch= in fichier!\n");
            return ret;
        }

        ret = LectureParametresdouble(fichier, "Delta_Cl=", "%lf", &Donnees->Delta_Cl);
        if(!ret)
        {
            printf("Can not find Delta_Cl= in fichier!\n");
            return ret;
        }


        ret = LectureParametresInt(fichier, "Confidence=", "%d", &Donnees->Confidence);
        if(!ret)
        {
            printf("Can not find Confidence= in fichier!\n");
            return ret;
        }


        ret = LectureParametresdouble(fichier, "ValidWidth=", "%lf", &Donnees->ValidWidth);
        if(!ret)
        {
            printf("Can not find ValidWidth= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Filter_cp=", "%d", &Donnees->Filter_cp);
        if(!ret)
        {
            printf("Can not find Filter_cp= in fichier!\n");
            return ret;
        }

        /* for Debug */
        ret = LectureParametresInt(fichier, "Debug=", "%d", &Donnees->Debug);
        if(!ret)
        {
            printf("Can not find Debug= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Aff_Zone=", "%d", &Donnees->Aff_Zone);
        if(!ret)
        {
            printf("Can not find Aff_Zone= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Aff_Median=", "%d", &Donnees->Aff_Median);
        if(!ret)
        {
            printf("Can not find Aff_Median= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "Aff_Result=", "%d", &Donnees->Aff_Result);
        if(!ret)
        {
            printf("Can not find Aff_Result= in fichier!\n");
            return ret;
        }

        /* for customer Init */
        ret = LectureParametresInt(fichier, "Custom_init=", "%d", &customInit);
        if(!ret)
        {
            printf("Can not find Custom_init= in fichier!\n");
            return ret;
        }

        ret = LectureParametresInt(fichier, "init_center=", "%d", &initCenter);
        if(!ret)
        {
            printf("Can not find init_center= in fichier!\n");
            return ret;
        }

        Donnees->carWidth = 1.8;
        Donnees->leftDeviation = 0.1;
        Donnees->rightDeviation = 0.1;
        Donnees->TTCWaring = 10;

        if(customInit)
        {
            ret = LectureParametresInt(fichierCustom, "CarWidth", "%d", &carWideth);
            if(!ret)
            {
                printf("Can not find CarWidth= in fichierCustom!\n");
                return ret;
            }
            Donnees->carWidth = (double)carWideth / 100;

            ret = LectureParametresInt(fichierCustom, "LeftDeviation", "%d", &leftDeviation);
            if(!ret)
            {
                printf("Can not find LeftDeviation= in fichierCustom!\n");
                return ret;
            }
            Donnees->leftDeviation = (double)leftDeviation / 100;

            ret = LectureParametresInt(fichierCustom, "RightDeviation", "%d", &rightDeviation);
            if(!ret)
            {
                printf("Can not find RightDeviation= in fichierCustom!\n");
                return ret;
            }
            Donnees->rightDeviation = (double)rightDeviation / 100;

            ret = LectureParametresInt(fichierCustom, "CameraHigh", "%d", &cameraHigh);
            if(!ret)
            {
                printf("Can not find CameraHigh= in fichierCustom!\n");
                return ret;
            }
            Donnees->Z0 = (double)cameraHigh / 100;

            ret = LectureParametresInt(fichierCustom, "RoadWidth", "%d", &roadWidth);
            if(!ret)
            {
                printf("Can not find RoadWidth= in fichierCustom!\n");
                return ret;
            }
            Donnees->Largeur = (double)roadWidth / 100;

            ret = LectureParametresInt(fichierCustom, "VehicleWarning", "%d", &vehicleWarning);
            if(!ret)
            {
                printf("Can not find VehicleWarning= in fichierCustom!\n");
                return ret;
            }
            Donnees->TTCWaring = (double)vehicleWarning;

            ret = LectureParametresInt(fichierCustom, "CameraPosx", "%d", &cameraPosx);
            if(!ret)
            {
                printf("Can not find CameraPosx= in fichierCustom!\n");
                return ret;
            }
            Donnees->X0 = (double)cameraPosx / 100;

            ret = LectureParametresInt(fichierCustom, "FirstLineY", "%d", &firstLineY);
            if(!ret)
            {
                printf("Can not find FirstLineY= in fichierCustom!\n");
                return ret;
            }

            ret = LectureParametresInt(fichierCustom, "SecondLineY", "%d", &secondLineY);
            if(!ret)
            {
                printf("Can not find SecondLineY= in fichierCustom!\n");
                return ret;
            }

            if(secondLineY > firstLineY)
            {
                Donnees->V_haut = firstLineY;
                Donnees->V_bas = secondLineY;
            }

            ret = LectureParametresInt(fichierCustom, "VerticalLineX", "%d", &verticalLineX);
            if(!ret)
            {
                printf("Can not find VerticalLineX= in fichierCustom!\n");
                return ret;
            }

            ret = LectureParametresInt(fichierCustom, "HorizontalLineY", "%d", &horizontalLineY);
            if(!ret)
            {
                printf("Can not find HorizontalLineY= in fichierCustom!\n");
                return ret;
            }

            if(initCenter)
            {
                Donnees->Cx = verticalLineX - Donnees->Eu * Donnees->Psi * A_TO_R;
                Donnees->Cy = horizontalLineY + Donnees->Ev * Donnees->Alpha * A_TO_R;
			    Donnees->Cx_N = Donnees->Cx;
                Donnees->Cy_N = Donnees->Cy;
            }
            else
            {
                Donnees->Psi = (double)(verticalLineX - Donnees->Cx) * R_TO_A / Donnees->Eu;
                Donnees->Alpha = (double)(Donnees->Cy - horizontalLineY) * R_TO_A / Donnees->Ev;
                printf("-----------hor:%d",horizontalLineY);
            }

			ret = LectureParametresInt(fichierCustom, "FCWSD_th", "%d", &Donnees->FCWSD_th);

            if(!ret)
            {
                printf("Can not find FCWSD_th= in fichierCustom!\n");
                return ret;
            }

        }

		Donnees->vanishPoint.x = Donnees->Cx_N + Donnees->Eu * Donnees->Psi * A_TO_R;
		Donnees->vanishPoint.y = Donnees->Cy_N - Donnees->Ev * Donnees->Alpha * A_TO_R;
		Donnees->Vangle = 0;
        return ret;

    } else {
        printf("fichier is NULL!\n");
        return 0;
    }

#else

        Donnees->runFlag = 0;
        Donnees->VEHICULE = 0;
        Donnees->MARQUEE = 1;
        Donnees->Eu_N = 835;
        Donnees->Ev_N = 569;
        Donnees->Tx_N = 1152;
        Donnees->Ty_N = 648;
        Donnees->Cx_N = 590;
        Donnees->Cy_N = 314;
        Donnees->Eu = Donnees->Eu_N;
        Donnees->Ev = Donnees->Ev_N;
        Donnees->Cx = Donnees->Cx_N;
        Donnees->Cy = Donnees->Cy_N;
        Donnees->Tx = Donnees->Tx_N;
        Donnees->Ty = Donnees->Ty_N;
        Donnees->Z0 = 1.2;
        Donnees->Largeur = 3.25;
        Donnees->X0 = 1.2;
        Donnees->Alpha = 0;
        Donnees->Psi = 0;
        Donnees->Ch = 0;
        Donnees->Cl = 0;
        Donnees->Dev_Largeur = 0.3;
        Donnees->Dev_X0 = 0.5;
        Donnees->Dev_Alpha = 5;
        Donnees->Dev_Psi = 5;
        Donnees->Dev_Ch = 0.01;
        Donnees->Dev_Cl = 0.00005;
        Donnees->V_haut = 340;
        Donnees->V_bas = 480;
        Donnees->H_zone = 10;
        Donnees->Var_Max = 70000;
        Donnees->Var_Min = 28;
        Donnees->Largeur_Zone_Min = 60;
        Donnees->Largeur_Zone_Max = 320;
        Donnees->Marge_Image = 5;
        Donnees->Nb_Pts_Min = 3;
        Donnees->Seuil_Grad = 1;
        Donnees->Grad_Core_Length = 8;
        Donnees->L_Bande_Sup = 6;
        Donnees->L_Bande_Inf = 20;
        Donnees->Nb_Tir_Med = 20;
        Donnees->Confiance = 1;
        Donnees->Ecart_Type = 9;
        Donnees->Residu_Max= 1000;
        Donnees->Coef_Median = 2;
        Donnees->Conf_Pts = 20;
        Donnees->Erreur_mini = 0.3;
        Donnees->Nb_Iterations = 100;
        Donnees->Proba = 0.08;
        Donnees->Proba_Bord = 0.04;
        Donnees->Tolerance = 0.001;
        Donnees->Lambda_Y = 0;
        Donnees->Delta_L = 0.1;
        Donnees->Delta_X0 = 0.1;
        Donnees->Delta_Alpha = 0.5;
        Donnees->Delta_Psi = 0.5;
        Donnees->Delta_Ch = 0.001;
        Donnees->Delta_Cl = 0.0004;
        Donnees->Confidence = 10;
        Donnees->ValidWidth = 0.1;
        Donnees->Filter_cp = 30;
        /* for Debug */
        Donnees->Debug = 1;
        Donnees->Aff_Zone = 0;
        Donnees->Aff_Median = 0;
        Donnees->Aff_Result = 0;
        /* for customer Init */
        customInit = 1;
        initCenter = 1;
        Donnees->carWidth = 1.6;
        Donnees->leftDeviation = 0.2;
        Donnees->rightDeviation = 0.2;
        Donnees->TTCWaring = 10;
        if(customInit)
        {
            carWideth = 160;
            Donnees->carWidth = (double)carWideth / 100;
           
            leftDeviation = 20;
            Donnees->leftDeviation = (double)leftDeviation / 100;

            rightDeviation = 20;
            Donnees->rightDeviation = (double)rightDeviation / 100;

            cameraHigh = 120;
            Donnees->Z0 = (double)cameraHigh / 100;

            roadWidth = 325;
            Donnees->Largeur = (double)roadWidth / 100;

            vehicleWarning = 10;
            Donnees->TTCWaring = (double)vehicleWarning;

            cameraPosx = 120;
            Donnees->X0 = (double)cameraPosx / 100;

            firstLineY = 340;
            secondLineY = 480;
            if(secondLineY > firstLineY)
            {
                Donnees->V_haut = firstLineY;
                Donnees->V_bas = secondLineY;
            }
            verticalLineX = 590;
            horizontalLineY =314;
            if(initCenter)
            {
                Donnees->Cx = verticalLineX - Donnees->Eu * Donnees->Psi * A_TO_R;
                Donnees->Cy = horizontalLineY + Donnees->Ev * Donnees->Alpha * A_TO_R;
            }
            else
            {
                Donnees->Psi = (double)(verticalLineX - Donnees->Cx) * R_TO_A / Donnees->Eu;
                Donnees->Alpha = (double)(Donnees->Cy - horizontalLineY) * R_TO_A / Donnees->Ev;
            }

        }

#endif

    return ret;
}

/*
   Function process:
   + Malloc the memory of Donnees
   Fan-in : 
   + Init_Modele_Fichier()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_Fichier()
*/
static void Allocation_Modele_Fichier(Fichier *Donnees)
{
    /* allocation memoires of structures based on parameters of fichier param.dat */
    Donnees->Compt_indice		= (int *)malloc(LDWS_NB_ZONES * sizeof(int));
    memset(Donnees->Compt_indice, 0, LDWS_NB_ZONES * sizeof(int));
    Donnees->templet			= (Templet *) malloc(LDWS_NB_ZONES * sizeof(Templet));
    memset(Donnees->templet, 0, LDWS_NB_ZONES * sizeof(Templet));

    Donnees->fInterFrameDisp	= (double *)malloc(LDWS_NB_ZONES * sizeof(double));
    memset(Donnees->fInterFrameDisp, 0, LDWS_NB_ZONES * sizeof(double));


    Donnees->Tab_Var		= (double *)malloc(LDWS_NB_ZONES * sizeof(double));	/* Variance 																			 */
    memset(Donnees->Tab_Var, 0, LDWS_NB_ZONES * sizeof(double));

    Donnees->Tab_Bool		= (int *)malloc(LDWS_NB_ZONES * sizeof(int));	/* Used to mark  the classified  area */
    memset(Donnees->Tab_Bool, 0, LDWS_NB_ZONES * sizeof(int));

    Donnees->tab_residus	= (double *)malloc(sizeof(double)* Donnees->Cx);
    memset(Donnees->tab_residus, 0, sizeof(double)* Donnees->Cx);


    Donnees->dataAve	= (Data_Stat *) malloc(LDWS_NB_BANDES * sizeof(Data_Stat));
    memset(Donnees->dataAve, 0, LDWS_NB_BANDES * sizeof(Data_Stat));

    Donnees->dataDiff	= (Data_Stat *) malloc(LDWS_NB_ZONES * sizeof(Data_Stat));
    memset(Donnees->dataDiff, 0, LDWS_NB_ZONES * sizeof(Data_Stat));

    Donnees->pCBPoint	= (Point *) malloc((CLIP_BSPLINE_POINT_NUM + 1) * sizeof(Point));
    memset(Donnees->pCBPoint, 0, (CLIP_BSPLINE_POINT_NUM + 1) * sizeof(Point));


    Donnees->M_A_J_Zone_X	= (double *)malloc(LDWS_NB_ZONES * sizeof(double));
    memset(Donnees->M_A_J_Zone_X, 0, LDWS_NB_ZONES * sizeof(double));

    Donnees->M_A_J_Zone_CX	= (double *)malloc(LDWS_NB_ZONES * LDWS_NB_ZONES * sizeof(double));
    memset(Donnees->M_A_J_Zone_CX, 0, LDWS_NB_ZONES * LDWS_NB_ZONES * sizeof(double));


    Donnees->M_A_J_Param_X		= (double *)malloc(Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Param_X, 0, Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Param_CX		= (double *)malloc(Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Param_CX, 0, Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));


    Donnees->M_A_J_Param_X_new	= (double *)malloc(Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Param_X_new, 0, Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Param_CX_new = (double *)malloc(Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Param_CX_new, 0, Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));


    Donnees->M_A_J_Suivi3_X_new		= (double *)malloc(LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_X_new, 0, LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_CX_new	= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_CX_new, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_CX_temp	= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_CX_temp, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_M_Suivi	= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_M_Suivi, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));


    Donnees->M_A_J_Suivi3_M			= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_M, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_M_T		= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_M_T, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_H			= (double *)malloc(LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_H, 0, LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Suivi3_H_T		= (double *)malloc(LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_H_T, 0, LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Suivi3_K_inv		= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_K_inv, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_K			= (double *)malloc(LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_K, 0, LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Suivi3_X_temp	= (double *)malloc(LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_X_temp, 0, LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_X_temp2	= (double *)malloc(LDWS_NB_PARAM * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_X_temp2, 0, LDWS_NB_PARAM * sizeof(double));

    Donnees->M_A_J_Suivi3_X			= (double *)malloc(Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_X, 0, Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Suivi3_CX		= (double *)malloc(Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_CX, 0, Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));

    Donnees->M_A_J_Suivi3_Temp		= (double *)malloc(LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));
    memset(Donnees->M_A_J_Suivi3_Temp, 0, LDWS_NB_PARAM * Donnees->NB_Z_P * sizeof(double));

	Donnees->edgeValue = (int *)malloc(Donnees->Tx * sizeof(int));
	memset(Donnees->edgeValue, 0, Donnees->Tx * sizeof(int));

	Donnees->edgeValueV = (int *)malloc(Donnees->Tx * sizeof(int));
	memset(Donnees->edgeValueV, 0, Donnees->Tx * sizeof(int));

}

/*
   Function process:
   + free the memory of Donnees
   Fan-in : 
   + Free_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: 
*/
void Free_Modele_Fichier(Fichier *Donnees)
{
    free(Donnees->Compt_indice);
    Donnees->Compt_indice = NULL;

    free(Donnees->templet);
    Donnees->templet = NULL;

    free(Donnees->fInterFrameDisp);
    Donnees->fInterFrameDisp = NULL;

    free(Donnees->Tab_Var);
    Donnees->Tab_Var = NULL;

    free(Donnees->Tab_Bool);
    Donnees->Tab_Bool = NULL;

    free(Donnees->tab_residus);
    Donnees->tab_residus = NULL;

    free(Donnees->dataAve);
    Donnees->dataAve = NULL;

    free(Donnees->dataDiff);
    Donnees->dataDiff = NULL;

    free(Donnees->pCBPoint);
    Donnees->pCBPoint = NULL;

    free(Donnees->M_A_J_Zone_X);
    Donnees->M_A_J_Zone_X = NULL;

    free(Donnees->M_A_J_Zone_CX);
    Donnees->M_A_J_Zone_CX = NULL;


    free(Donnees->M_A_J_Param_X);
    Donnees->M_A_J_Param_X = NULL;

    free(Donnees->M_A_J_Param_CX);
    Donnees->M_A_J_Param_CX = NULL;

    free(Donnees->M_A_J_Param_X_new);
    Donnees->M_A_J_Param_X_new = NULL;

    free(Donnees->M_A_J_Param_CX_new);
    Donnees->M_A_J_Param_CX_new = NULL;


    free(Donnees->M_A_J_Suivi3_X_new);
    Donnees->M_A_J_Suivi3_X_new = NULL;

    free(Donnees->M_A_J_Suivi3_CX_new);
    Donnees->M_A_J_Suivi3_CX_new = NULL;

    free(Donnees->M_A_J_Suivi3_CX_temp);
    Donnees->M_A_J_Suivi3_CX_temp = NULL;

    free(Donnees->M_A_J_Suivi3_M_Suivi);
    Donnees->M_A_J_Suivi3_M_Suivi = NULL;

    free(Donnees->M_A_J_Suivi3_M);
    Donnees->M_A_J_Suivi3_M = NULL;

    free(Donnees->M_A_J_Suivi3_M_T);
    Donnees->M_A_J_Suivi3_M_T = NULL;

    free(Donnees->M_A_J_Suivi3_H);
    Donnees->M_A_J_Suivi3_H = NULL;

    free(Donnees->M_A_J_Suivi3_H_T);
    Donnees->M_A_J_Suivi3_H_T = NULL;

    free(Donnees->M_A_J_Suivi3_K_inv);
    Donnees->M_A_J_Suivi3_K_inv = NULL;

    free(Donnees->M_A_J_Suivi3_K);
    Donnees->M_A_J_Suivi3_K = NULL;

    free(Donnees->M_A_J_Suivi3_X_temp);
    Donnees->M_A_J_Suivi3_X_temp = NULL;

    free(Donnees->M_A_J_Suivi3_X_temp2);
    Donnees->M_A_J_Suivi3_X_temp2 = NULL;

    free(Donnees->M_A_J_Suivi3_X);
    Donnees->M_A_J_Suivi3_X = NULL;

    free(Donnees->M_A_J_Suivi3_CX);
    Donnees->M_A_J_Suivi3_CX = NULL;

    free(Donnees->M_A_J_Suivi3_Temp);
    Donnees->M_A_J_Suivi3_Temp = NULL;

        free(Donnees->edgeValue);
	Donnees->edgeValue = NULL;

        free(Donnees->edgeValueV);
	Donnees->edgeValueV = NULL;

    free(Donnees);
    Donnees = NULL;

}

/*
   Function process:
   + Malloc the memory of M_3D_new
   Fan-in : 
   + Allocation_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_3D()
*/
static void Allocation_Modele_3D(Modele_3D * M_3D_new, const Fichier *Donnees)
{
    M_3D_new->V			= (int *)malloc(LDWS_NB_ZONES * sizeof(int));	/* Xd */
    memset(M_3D_new->V, 0, LDWS_NB_ZONES * sizeof(int));

    M_3D_new->X			= (double *)malloc(Donnees->NB_Z_P * sizeof(double));	/* X */
    memset(M_3D_new->X, 0, Donnees->NB_Z_P * sizeof(double));

    M_3D_new->CX		= (double *)malloc(Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));	/* Cx */
    memset(M_3D_new->CX, 0, Donnees->NB_Z_P * Donnees->NB_Z_P * sizeof(double));

    M_3D_new->Param		= (double *)malloc(LDWS_NB_PARAM * sizeof(double));	/* Xl */
    memset(M_3D_new->Param, 0, LDWS_NB_PARAM * sizeof(double));

    M_3D_new->C_Param	= (double *)malloc(LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));	/* Cxl */
    memset(M_3D_new->C_Param, 0, LDWS_NB_PARAM * LDWS_NB_PARAM * sizeof(double));

}

/*
   Function process:
   + free the memory of M_3D
   Fan-in : 
   + Free_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: 
*/
static void Free_Modele_3D(Modele_3D * M_3D)
{
    /* desallocation des structures du modele image */
    free(M_3D->V);
    M_3D->V = NULL;

    free(M_3D->X);
    M_3D->X = NULL;

    free(M_3D->CX);
    M_3D->CX = NULL;

    free(M_3D->Param);
    M_3D->Param = NULL;

    free(M_3D->C_Param);
    M_3D->C_Param = NULL;

    free(M_3D);
    M_3D = NULL;

}

/*
   Function process:
   + Malloc the memory of M_Filter
   Fan-in : 
   + Allocation_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_Filter()
*/
static void Allocation_Modele_Filter(Modele_Filter *M_Filter, const Fichier *Donnees)
{
    M_Filter->dataSrc		= (Data_Stat *) malloc(LDWS_NB_ZONES * sizeof(Data_Stat));
    memset(M_Filter->dataSrc, 0, LDWS_NB_ZONES * sizeof(Data_Stat));

    M_Filter->dataAve		= (Data_Stat *) malloc(LDWS_NB_BANDES * sizeof(Data_Stat));
    memset(M_Filter->dataAve, 0, LDWS_NB_BANDES * sizeof(Data_Stat));

    M_Filter->dataAveDiff	= (Data_Stat *) malloc(LDWS_NB_BANDES * sizeof(Data_Stat));
    memset(M_Filter->dataAveDiff, 0, LDWS_NB_BANDES * sizeof(Data_Stat));

    M_Filter->templet		= (Templet *) malloc(LDWS_NB_ZONES * sizeof(Templet));
    memset(M_Filter->templet, 0, LDWS_NB_ZONES * sizeof(Templet));

}

/*
   Function process:
   + free the memory of M_Filter
   Fan-in : 
   + Free_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: 
*/
static void Free_Modele_Filter(Modele_Filter *M_Filter)
{
    /* desallocation des structures du modele image */
    free(M_Filter->dataSrc);
    M_Filter->dataSrc = NULL;

    free(M_Filter->dataAve);
    M_Filter->dataAve = NULL;

    free(M_Filter->dataAveDiff);
    M_Filter->dataAveDiff = NULL;

    free(M_Filter->templet);
    M_Filter->templet = NULL;

    free(M_Filter);
    M_Filter = NULL;

}

/*
   Function process:
   + Malloc the memory of L_output
   Fan-in : 
   + Init_Modele_Output()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_OutPut()
*/
static void Allocation_Modele_OutPut(LDWS_Output *L_output, const Fichier *Donnees)
{
    L_output->pPoint	= (LDWS_Point *) malloc(L_output->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));
    memset(L_output->pPoint, 0, L_output->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));

    L_output->pCaPoint	= (LDWS_Point *) malloc(L_output->NB_INTERVALLES * LDWS_NB_BANDES * sizeof(LDWS_Point));
    memset(L_output->pCaPoint, 0, L_output->NB_INTERVALLES * LDWS_NB_BANDES * sizeof(LDWS_Point));

    L_output->Param		= (double *)malloc(LDWS_NB_PARAM * sizeof(double));
    memset(L_output->Param, 0, LDWS_NB_PARAM * sizeof(double));

}

/*
   Function process:
   + free the memory of L_output
   Fan-in : 
   + LDWS_Finalization()
   Fan-out:
   + N/A
ATTENTION: 
*/
void Free_Modele_OutPut(LDWS_Output *L_output)
{
    /* desallocation des structures du modele image */
    free(L_output->pPoint);
    L_output->pPoint = NULL;

    free(L_output->pCaPoint);
    L_output->pCaPoint = NULL;

    free(L_output->Param);
    L_output->Param = NULL;

    L_output = NULL;

}

/*
   Function process:
   + Malloc the memory of 3D Model struct
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_Struct()
*/
void Allocation_Modele_Struct(Modele_3D *M_3D_init, Modele_3D *M_3D_est, 
        Modele_Filter *M_Filter, const Fichier *Donnees)
{
    /* Modele 3D initiale */
    Allocation_Modele_3D(M_3D_init, Donnees);

    /* Modele 3D est */
    Allocation_Modele_3D(M_3D_est, Donnees);

    Allocation_Modele_Filter(M_Filter, Donnees);
}

/*
   Function process:
   + free the memory of M_3D_init,M_3D_est and M_Filter
   Fan-in : 
   + LDWS_Finalization()
   Fan-out:
   + N/A
ATTENTION: 
*/
void Free_Modele_Struct(Modele_3D *M_3D_init, Modele_3D *M_3D_est, Modele_Filter *M_Filter)
{
    Free_Modele_Filter(M_Filter);

    Free_Modele_3D(M_3D_est);

    Free_Modele_3D(M_3D_init);
}

/*
   Function process:
   + Init Donnees
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + Allocation_Modele_Fichier()
ATTENTION: __________
*/
void Init_Modele_Fichier(Fichier *Donnees)
{
    int i;

    /* Converted to radians  */
    Donnees->Alpha		= Donnees->Alpha * A_TO_R;
    Donnees->Psi		= Donnees->Psi * A_TO_R;
    Donnees->Dev_Alpha	= Donnees->Dev_Alpha * A_TO_R;
    Donnees->Dev_Psi	= Donnees->Dev_Psi * A_TO_R;

    Donnees->Delta_Alpha	= Donnees->Delta_Alpha * A_TO_R;
    Donnees->Delta_Psi		= Donnees->Delta_Psi * A_TO_R;
    /* Donnees->Delta_Delta = Donnees->Delta_Delta * A_TO_R; */

    Donnees->NB_Z_P			= LDWS_NB_ZONES + LDWS_NB_PARAM;
    Donnees->NB_INTERVALLES = LDWS_NB_ZONES / LDWS_NB_BANDES;

    Donnees->flgStatus = ROAD_NORMAL;

    Donnees->Seuil_Grad *= Donnees->Grad_Core_Length;

    Donnees->Grad					= 0;

	for(i = 0; i < 6; i++)
	{
       Donnees->Confidence_detection[i]	= 0;
	}

    Donnees->Vh = Donnees->Cy -  Donnees->Ev * Donnees->Alpha;

    Donnees->Suivi	= 1;			/* 4 */
    Donnees->Init	= 0;
    Donnees->k		= Donnees->Eu / (Donnees->Ev * Donnees->Z0);

    Allocation_Modele_Fichier(Donnees);

    for (i = 0; i < LDWS_NB_ZONES; ++i)
    {
        Donnees->templet[i].iGrad = Donnees->Seuil_Grad;
        Donnees->templet[i].fYAve = MIN_LING_AVE_Y;
    }
}

/*
   Function process:
   + Init the value of M_3D_init->Param and M_3D_init->C_Param
   Fan-in : 
   + Allocation_Modele_Struct()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_Filter()
*/
static void Modele_VELAC(Modele_3D *M_3D_init, const Fichier *Donnees)
{
    int i, j;

    /*
     * initialisation du vecteur moyen et covariance des param鑤res pour velac 
     */
    for (j = 0; j < LDWS_NB_PARAM; j++)
    {
        for (i = 0; i < LDWS_NB_PARAM; i++)
        {
            M_3D_init->C_Param[i + j * LDWS_NB_PARAM] = 0;
        }
    }

    M_3D_init->Param[0] = Donnees->Largeur;
    M_3D_init->Param[1] = Donnees->X0;
    M_3D_init->Param[2] = Donnees->Psi;
    M_3D_init->Param[3] = Donnees->Alpha;
    M_3D_init->Param[4] = Donnees->Ch;
    M_3D_init->Param[5] = Donnees->Cl;

    M_3D_init->C_Param[0 * LDWS_NB_PARAM]	  = Donnees->Dev_Largeur * Donnees->Dev_Largeur;
    M_3D_init->C_Param[1 * LDWS_NB_PARAM + 1] = Donnees->Dev_X0 * Donnees->Dev_X0;
    M_3D_init->C_Param[2 * LDWS_NB_PARAM + 2] = Donnees->Dev_Psi * Donnees->Dev_Psi;
    M_3D_init->C_Param[3 * LDWS_NB_PARAM + 3] = Donnees->Dev_Alpha * Donnees->Dev_Alpha;
    M_3D_init->C_Param[4 * LDWS_NB_PARAM + 4] = Donnees->Dev_Ch * Donnees->Dev_Ch;
    M_3D_init->C_Param[5 * LDWS_NB_PARAM + 5] = Donnees->Dev_Cl * Donnees->Dev_Cl;
}

void Modele_TRACTEUR(Modele_3D *M_3D_init, Fichier *Donnees)
{
    int i, j;

    /*
     * initialisation du vecteur moyen et covariance des param鑤res pour velac 
     */
    for (j = 0; j < LDWS_NB_PARAM; j++)
    {
        for (i = 0; i < LDWS_NB_PARAM; i++)
        {
            M_3D_init->C_Param[i + j * LDWS_NB_PARAM] = 0;
        }
    }

    M_3D_init->Param[0] = Donnees->Largeur;
    M_3D_init->Param[1] = Donnees->X0;
    M_3D_init->Param[2] = Donnees->Psi;
    M_3D_init->Param[3] = Donnees->Alpha;
    M_3D_init->Param[4] = Donnees->Ch;

    /*
     *  M_3D_init->Param[5] = Donnees->A * (Donnees->X0 - Donnees->X0c) + Donnees->B * Donnees->Psi;
     */
    M_3D_init->C_Param[0 * LDWS_NB_PARAM]	  = Donnees->Dev_Largeur * Donnees->Dev_Largeur;
    M_3D_init->C_Param[1 * LDWS_NB_PARAM + 1] = Donnees->Dev_X0 * Donnees->Dev_X0;
    M_3D_init->C_Param[2 * LDWS_NB_PARAM + 2] = Donnees->Dev_Psi * Donnees->Dev_Psi;
    M_3D_init->C_Param[3 * LDWS_NB_PARAM + 3] = Donnees->Dev_Alpha * Donnees->Dev_Alpha;
    M_3D_init->C_Param[4 * LDWS_NB_PARAM + 4] = Donnees->Dev_Ch * Donnees->Dev_Ch;

    /*
     *  M_3D_init->C_Param[5 * LDWS_NB_PARAM + 5]  = Donnees->A * Donnees->A * Donnees->Dev_X0 * Donnees->Dev_X0 + Donnees->B * Donnees->B * Donnees->Dev_Psi * Donnees->Dev_Psi;
     *
     *  M_3D_init->C_Param[1 * LDWS_NB_PARAM + 5]  = M_3D_init->C_Param[5 * LDWS_NB_PARAM + 1] = Donnees->A * Donnees->Dev_X0 * Donnees->Dev_X0;
     *  M_3D_init->C_Param[2 * LDWS_NB_PARAM + 5]  = M_3D_init->C_Param[5 * LDWS_NB_PARAM + 2] = Donnees->B * Donnees->Dev_Psi * Donnees->Dev_Psi;
     */
}

/*
   Function process:
   + In the initialization, given the value of M_3D_init->Param and M_3D_init->C_Param;
   If lines not found, Init the M_I_init->X and CX based on M_3D_init
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + Modele_VELAC()
ATTENTION: All the malloc free in Free_Modele_3D()
*/
void Init_Modele_3D(Modele_Image *M_I_init, Modele_3D *M_3D_init, 
        LDWS_Output *L_output, Fichier *Donnees)
{
    int i, j;
    double rapport;

    if (Donnees->Init == 0)
    {
        if (Donnees->VEHICULE == VELAC)
        {
            Modele_VELAC(M_3D_init, Donnees);
        }
        if (Donnees->VEHICULE == TRACTEUR)
        {
            Modele_TRACTEUR(M_3D_init, Donnees);
        }
    }
    else
    {
        rapport = (double)Donnees->Confidence_detection[0] / (double)Donnees->Confidence;
        for (j = 0; j < LDWS_NB_ZONES; j++)
        {
            M_I_init->X[j] = (rapport * M_I_init->X[j]) + ((1 - rapport) * M_3D_init->X[j]);
            for (i = 0; i < LDWS_NB_ZONES; i++)
            {
                M_I_init->CX[j * LDWS_NB_ZONES + i] =
                    (rapport * M_I_init->CX[j * LDWS_NB_ZONES + i])
                    + ((1 - rapport) * M_3D_init->CX[j * Donnees->NB_Z_P + i]);
            }
        }

        for (i = 0; i < LDWS_NB_PARAM; ++i)
        {
            L_output->Param[i] = (1 - rapport) * M_3D_init->Param[i] + L_output->Param[i] * rapport;
        }

        Donnees->Vh = (int)((1 - rapport) * (Donnees->Cy - Donnees->Ev * L_output->Param[3]) + Donnees->Vh * rapport + 0.5);

        for (i = 0; i < LDWS_NB_ZONES; ++i)
        {
            L_output->pCaPoint[i].x = (int)(M_I_init->X[i] + 0.5);
        }
    }
}

/*
   Function process:
   + Init the related parameters in M_I_init and M_I_est in each frame
   Fan-in : 
   + LDWS_RoadTracker()
   Fan-out:
   + N/A()
ATTENTION: 
*/
void Init_Modele_Image(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees)
{
    //int i;

    /* initialisation des diffXerents parametres */
    /*for (i = 0; i < LDWS_NB_ZONES; i++)
    {
        M_I_init->Zones_Testees[i]		= 0;
        M_I_init->Zones_Detectees[i]	= 0;
        Donnees->Compt_indice[i]		= 0;
    }*/
	memset(M_I_init->Zones_Testees,0,sizeof(char) * LDWS_NB_ZONES);
	memset(M_I_init->Zones_Detectees,0,sizeof(char) * LDWS_NB_ZONES);
	memset(Donnees->Compt_indice,0,sizeof(int) * LDWS_NB_ZONES);

    memset(M_I_init->templet, 0, sizeof(Templet) * LDWS_NB_ZONES);

    /*for (i = 0; i < LDWS_NB_ZONES << 1; i++)
    {
        M_I_init->Xi[i]		= 0;
        M_I_init->CXi[i]	= 0;
    }*/
	memset(M_I_init->Xi,0,sizeof(double) * (LDWS_NB_ZONES << 1));
	memset(M_I_init->CXi,0,sizeof(double) * (LDWS_NB_ZONES << 1));

	M_I_init->Route		= 0;
	M_I_init->Route_half = 0;
	M_I_init->RouteL	= 0;
	M_I_init->RouteR	= 0;
	M_I_init->Nb_Pts	= 0;
	M_I_init->Nb_Pts_L	= 0;
	M_I_init->Nb_Pts_R	= 0;
	M_I_init->Proba		= 0;
	M_I_init->Proba_L	= 0;
	M_I_init->Proba_R	= 0;

	/*for (i = 0; i < LDWS_NB_BANDES; i++)
	{
		M_I_init->Proba_Bord[i] = 0;
		M_I_init->Grad_Bord[i]	= 0;
		M_I_init->Proba_Bord_half[i] = 0;
	}*/
	memset(M_I_init->Proba_Bord,0,sizeof(double) * LDWS_NB_BANDES);
	memset(M_I_init->Grad_Bord,0,sizeof(int) * LDWS_NB_BANDES);
	memset(M_I_init->Proba_Bord_half,0,sizeof(double) * LDWS_NB_BANDES);

	M_I_est->Route	= 0;
	M_I_est->Route_half = 0;
	M_I_est->RouteL	= 0;
	M_I_est->RouteR	= 0;
	M_I_est->Nb_Pts = 0;
	M_I_est->Nb_Pts_L = 0;
	M_I_est->Nb_Pts_R = 0;
	M_I_est->Proba	  = 0;
	M_I_est->Proba_L  = 0;
	M_I_est->Proba_R  = 0;

	/*for (i = 0; i < LDWS_NB_BANDES; i++)
	{
		M_I_est->Proba_Bord[i]	= 0;
		M_I_est->Grad_Bord[i]	= 0;
		M_I_est->Proba_Bord_half[i] = 0;
	}*/
	memset(M_I_est->Proba_Bord,0,sizeof(double) * LDWS_NB_BANDES);
	memset(M_I_est->Grad_Bord,0,sizeof(int) * LDWS_NB_BANDES);
	memset(M_I_est->Proba_Bord_half,0,sizeof(double) * LDWS_NB_BANDES);

    Donnees->Compteur = 0;
}

/*
   Function process:
   + Init the values of M_Filter
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + N/A
ATTENTION: All the malloc free in Free_Modele_3D()
*/
void Init_Modele_Filter(Modele_Filter *M_Filter, const Fichier *Donnees)
{
    int i;

    M_Filter->detect_flg = 0;
	M_Filter->Route = 0;

    for (i = 0; i < LDWS_NB_ZONES; ++i)
    {
        M_Filter->dataSrc[i].dataX	= INVAILD_TAN;
        M_Filter->dataSrc[i].dataA	= INVAILD_TAN;
        M_Filter->templet[i].iGrad	= INVAILD_TAN;
        M_Filter->templet[i].fAngle = INVAILD_TAN;
        M_Filter->templet[i].fYAve	= INVAILD_TAN;
    }

    for (i = 0; i < LDWS_NB_BANDES; ++i)
    {
        M_Filter->dataAve[i].dataX		= INVAILD_TAN;
        M_Filter->dataAve[i].dataA		= INVAILD_TAN;
        M_Filter->dataAveDiff[i].dataX	= INVAILD_TAN;
        M_Filter->dataAveDiff[i].dataA	= INVAILD_TAN;
    }
}

/*
   Function process:
   + Init the output of LDWS
   Fan-in : 
   + LDWS_InitRoadTracker()
   Fan-out:
   + Modele_VELAC()
ATTENTION: All the malloc free in Free_Modele_3D()
*/
void Init_Modele_Output(LDWS_Output *L_output, const Fichier *Donnees, const Modele_3D *M_3D_init)
{
    int i;

	L_output->Route					= 0;
	L_output->Route_half			= 0;
	L_output->Route_L					= 0;
	L_output->Route_R					= 0;
	for(i = 0; i < 6; i++)
	{
	    L_output->Confidence_detection[i]	= 0;
	}
	L_output->Confidence			= Donnees->Confidence;
	L_output->NB_INTERVALLES		= Donnees->NB_INTERVALLES;
	L_output->NB_BANDES				= LDWS_NB_BANDES;
	L_output->Ev					= Donnees->Ev;
	L_output->Z0					= Donnees->Z0;
	L_output->Tx					= Donnees->Tx;
	L_output->Ty					= Donnees->Ty;
	L_output->LPointLength			= (CBSPLINE_POINT_NUM + Donnees->NB_INTERVALLES - CLIP_BSPLINE_POINT_NUM);

    Allocation_Modele_OutPut(L_output, Donnees);

    for (i = 0; i < LDWS_NB_PARAM; ++i)
    {
        L_output->Param[i] = M_3D_init->Param[i];
    }
}
