#ifndef _DATA_H_
#define _DATA_H_

//#define _LDWS_DEBUG_ /* in order to debug algorithm,and need opencv */
//#define _LDWS_detail_DEBUG_
//#define _NEON_
//#define _NE10_

#include "LDWS_Interface.h"

#ifdef _LDWS_DEBUG_
	#ifdef WIN32
		#define _WIN32_LDWS_DEBUG_
	#else
		#define _ANDROID_LDWS_DEBUG_
	#endif
#endif


#ifdef _NEON_
    #include <arm_neon.h>
#endif

#ifndef ABS

	#define ABS( x ) ( (x) > 0 ? (x) : (-(x) ) )
#endif

#ifndef PI
	#define PI          (3.141592)
	#define A_TO_R      (0.017453)
	#define R_TO_A      (57.295791)
#endif

#define VELAC			(0)
#define TRACTEUR		(1)

#define ROAD_NORMAL		(0)
#define ROAD_CHANGE		(1)

#define STATIQUE		(0)
#define DYNAMIQUE		(1)

#ifdef _WIN32_LDWS_DEBUG_

	#define M_moy		(0)
	#define M_cov		(1)
	#define M_cbsmoy	(2)
#endif

#define INVAILD_TAN				(10000)

#define LDWS_NB_ZONES			(18)
#define LDWS_NB_BANDES			(2)
#define LDWS_NB_PARAM			(6)

#define DELTA_ANGLE_FRAME       (0.2)
#define DELTA_X_FRAME           (10)

#define LDWS_MAX_ROI_HEIGHT		(108) 

/********Lane Point ***********/
#define DELTA_LANE_WIDTH_WIDE	(0.6)
#define MAX_LANE_ANGLE          (1.3)
#define MIN_LING_AVE_Y          (70)
#define DOWN_TH_GRAD            (0.7)
#define LDWS_OUTPUT_CBP_NUM     (10)
#define CLIP_BSPLINE_POINT_NUM	(6)
#define CBSPLINE_POINT_NUM      (LDWS_OUTPUT_CBP_NUM * (CLIP_BSPLINE_POINT_NUM + 1 - 3) )

typedef struct 
{
	double fAngle;
	double fYAve;
	int iGrad;
} Templet;

typedef struct 
{
	double x;
	double y;
} Point;

typedef struct 
{
	double dataX;
	double dataA;
} Data_Stat;

/*
 * structure permettant de receuillir toutes les donnes necessaires a
 * l'agorithme il est remplit par defaut par le fichier param.dat 
 */
typedef struct 
{
	double Z0;					/* hauteur de la camera */
    double k;
	double kWidth;
	double Erreur_mini;         
	double Confiance;			
	double Ecart_Type;			
	double Coef_Median;		
							
	double Proba;				/* valeur de la probabilite pour que la route
								 * soit retrouvee */
	double Proba_Bord;			/* valeur de la probabilite pour qu'un bord de
								 * la route soit retrouve */
	double Tolerance;			/* tolerance de mesure sur la probabilite */
	/*---       sur les parametres qui evoluent                 ---*/

	double Largeur;				/* valeur moyenne dela largeur de la route
								 * (3.5) */
	double Dev_Largeur;			/* ecart type sur la Largeur de la route
								 * (0.25) */
	double X0;					/* valeur moyenne de la position du vehicule
								 * bord droit (1.75) */
	double Dev_X0;				/* ecart type sur la position du vehicule
								 * (1.75) */
	double Psi;					/* valeur moyenne de Psi (0) */
	double Dev_Psi;				/* ecart type sur Psi (6) */
	double Alpha;				/* valeur moyenne de Alpha pour le zoom 1000
								 * 4.5 */
	double Dev_Alpha;			/* ecart type sur Alpha (1) */
	double Ch;					/* valeur moyenne de la Courbure horizontale
								 * (0) */
	double Dev_Ch;				/* ecart type sur la Courbure horizontale
								 * (0.006) */
	double Cl;
	double Dev_Cl;				/* ecart type sur la Courbure horizontale
								 * (0.006) */

	/*---- confiance pour le suivi sur chacune des parametres ------*/
	/*
	 * double Delta; 
	 */
	double Lambda_Y;
	double Delta_L;
	double Delta_X0;
	double Delta_Psi;
	double Delta_Alpha;
	double Delta_Ch;
	double Delta_Cl;
	
	double ValidWidth;

	double carWidth;
	double leftDeviation;
	double rightDeviation;
	double TTCWaring;

	double *fInterFrameDisp;

	int VEHICULE;				/* The vheicle type, 0 for vehicle and 1 for TRACTEUR */
	int MARQUEE;				/* The road type (marquee=1 non marquee=0) */
	int NB_Z_P;					/* The addition of NB_ZONES and NB_PARAMS */
	int NB_INTERVALLES;			/* Num of points of each lines */

	/************ initalisation.c ***************/
	int Init;
	int runFlag;                /* 0 run on narrow view; 1 run on wide view */
	int V_haut;					/* ordonees max et min */
	int V_bas;
	int H_zone;					/* hauteur de la premire zone */
	int Eu;						/* eu et ev pour un zoom donne */
	int Ev;
	int Tx;						/* taille de l image enx et en y */
	int Ty;
	int Vh;

	int Cx;
	int Cy;

	int Eu_N;
	int Ev_N;
	int Tx_N;						
	int Ty_N;
	int Cx_N;
	int Cy_N;

	/************ detection zone.c *****************/
	int Var_Max;				/* variance max des zones de recherche */
	int Var_Min;				/* variance min des zones de recherche */
	int Largeur_Zone_Min;		/* The mini width of detect zone */
	int Largeur_Zone_Max;       /* The maxi width of detect zone */
	int Marge_Image;			/* The extend value of left and right marge of image 图像左右边界扩充像素值 */
	int Nb_Pts_Min;				/* Mini Num of points to detected in zone */

	/*************** c************************/
	int Grad_Core_Length;
	int Grad;
	int Seuil_Grad;				/* seuil pour le gradient --TH */
	int L_Bande_Sup;
	int L_Bande_Inf;

        int *edgeValue;
        int *edgeValueV;

	Templet *templet;

	/************* median.c *****************/
	int Nb_Tir_Med;				
	int Residu_Max;				/* valeur maximal du residu --LS max Residual */
	int Conf_Pts;				/* confiance sur les points dtects --detect
								 * confidence of points */

	/************* recherche.c**************/
	int Compteur;				/* compteur d'iterations iterator cnt */
	int Arbre;					/* niveau dans larbre de recherche --seach tree 
								 * level --deep p */
	int *Compt_indice;			/* nombre d'iteration effectue pour chaque
								 * branche de l'arbre --The number of
								 * iteration for each branch of the tree */
	int Nb_Iterations;			/* nombre d'iterations maximales dans
								 * l'arbre de recherche -- */
	int Suivi;					/* permet de fait diminuer ou augmenter le
								 * nombre d'iteration de la recherche */

	/*--- mise en place de valeurs moyennes et des ecarts-types ---*/

    /*
	 * double Delta_Delta; 
	 */

	int Filter_cp;
	int Confidence_detection[6];
	int Confidence;
	int flgStatus;

	double *Tab_Var;			/* Variance */
	int *Tab_Bool;
	double *tab_residus;
	Data_Stat *dataAve;
	Data_Stat *dataDiff;

	Point *pCBPoint;

	double *M_A_J_Zone_X;
	double *M_A_J_Zone_CX;

	double *M_A_J_Param_X;
	double *M_A_J_Param_CX;
	double *M_A_J_Param_X_new;
	double *M_A_J_Param_CX_new;

	double *M_A_J_Suivi3_X_new;
	double *M_A_J_Suivi3_CX_new;
	double *M_A_J_Suivi3_CX_temp;
	double *M_A_J_Suivi3_M_Suivi;
	double *M_A_J_Suivi3_M;
	double *M_A_J_Suivi3_M_T;

	double *M_A_J_Suivi3_H;
	double *M_A_J_Suivi3_H_T;
	double *M_A_J_Suivi3_K_inv;
	double *M_A_J_Suivi3_K;
	double *M_A_J_Suivi3_X_temp;
	double *M_A_J_Suivi3_X_temp2;

	double *M_A_J_Suivi3_X;
	double *M_A_J_Suivi3_CX;
	double *M_A_J_Suivi3_Temp;


	/*---- paramtres pour l'estimation de delta -----*/
	/*
	 * double   A;      / * coefficient A et B pour la loi de commande* /
	 * double   B;
	 * double   X0c;    / * position laterale voulue sur la route* /
	 */

	/*---- paramtres pour le dboguage ----*/
	int Debug;
	int Aff_Zone;
	int Aff_Median;
	int Aff_Result;

	int changeLine;
	int FCWSD_th;

	LDWS_Point vanishPoint;
	double Vangle;

} Fichier;

/*
 * image model Xd 
 */
typedef struct 
{

	double Proba;								/* The proportion for both two sides */
	double Proba_L;
	double Proba_R;
	double Proba_Bord[LDWS_NB_BANDES];			/* The proportion for one side */
	double Proba_Bord_half[LDWS_NB_BANDES];			/* The proportion for one side */
	
	double X[LDWS_NB_ZONES];					/* ui */
	double CX[LDWS_NB_ZONES * LDWS_NB_ZONES];

	double Xi[LDWS_NB_ZONES << 1];
	double CXi[LDWS_NB_ZONES << 1];

	Templet templet[LDWS_NB_ZONES];

	int V[LDWS_NB_ZONES];						/* vi */
	int Grad_Bord[LDWS_NB_BANDES];
	
	int Nb_Pts;									/* edge points total number */
	int Nb_Pts_L;	
	int Nb_Pts_R;	

	char Zones_Testees[LDWS_NB_ZONES];			/* */
	char Zones_Detectees[LDWS_NB_ZONES];		/* detect flg */

	char Route;
	char Route_half;
	char RouteL;
	char RouteR;
} Modele_Image;

/*
 * 3D model X 
 */
typedef struct 
{
	double *X;					/* X ui */
	double *CX;					/* Cx */
	double *Param;				/* Xl */
	double *C_Param;			/* Clp */
	int *V;						/* vi */
} Modele_3D;

/*
 * ROI 
 */
typedef struct 
{
	double X[2];				/* ui */
	double CX[3];				/* Cx */
	Templet measure;             
	int sampleK;
	int V[2];					/* vi */
	int RFlag;                  /* 0-Zone belongs to left line; 1-belongs to right line*/
	int indice_zone;
} Zone;

typedef struct 
{
	Data_Stat	*dataSrc;
	Data_Stat	*dataAve;
	Data_Stat	*dataAveDiff;
	Templet		*templet;
	char detect_flg;
	char Route;
} Modele_Filter;
#endif
