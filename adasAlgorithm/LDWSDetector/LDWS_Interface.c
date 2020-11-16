#include <stdlib.h>
#include <math.h>
#include <string.h> 

#include "data.h"
#include "LDWS_Interface.h"
#include "Road_Tracker.h"
#include "Init_Struct.h"
#include "LDWS_AlarmDecision.h"

const char* LIB_INFO = "libsmarteye.a version: 1.0.2 (2018-04-28)";

#ifdef _WIN32_LDWS_DEBUG_

Fichier *Donnees;
Modele_Image M_I_init;
Modele_Image M_I_est;
Modele_3D *M_3D_init;
Modele_3D *M_3D_est;
Modele_Filter *M_Filter;
LDWS_Output *L_output;
LDWS_Output *L_output_N;
LDWS_VPoint pLDWSVPoint;
#else

static Fichier *Donnees;
static Modele_Image M_I_init;
static Modele_Image M_I_est;
static Modele_3D *M_3D_init;
static Modele_3D *M_3D_est;
static Modele_Filter *M_Filter;
static LDWS_Output *L_output;
static LDWS_Output *L_output_N;
LDWS_VPoint pLDWSVPoint;
#endif

/*
 Realized function:
 + malloc the memory for parameters
 */
void LDWS_AllocModel(void);

void LDWS_Init(const char *fichier_init, const char *fichier_init_custom);

void LDWS_Tracker(const unsigned char *Tab_Image);

void LDWS_GetResult(LDWS_Output **pLDWSOutput);

void LDWS_GetXYZofWorldfromImage(int img_x, int img_y, const int Cx,
		const int Cy, const double H, const int f, const double alpha,
		double *world_X, double *world_Y, double *world_Z);

void LDWS_FreeResult(LDWS_Output **pLDWSOutput);

int LDWS_Get_RunFlag(void);

double LDWS_GetCarWidth(void);

double LDWS_GetLeftDeviation(void);

double LDWS_GetRightDeviation(void);

double LDWS_GetWarningTTC(void);

void LDWS_ChangeResultWtoN(void);

void LDWS_GetxyofImagefromWorld(int *img_x, int *img_y, int Cx, int Cy,
		double H, int f, double alpha, double world_X, double world_Y,
		double world_Z);

void LDWS_Getinit(LDWS_InitGuid **pLDWSInit);

void LDWS_Freeinit(LDWS_InitGuid **pLDWSInit);

int LDWS_GetVanishY(void);

int LDWS_GetVanishY_N(void);

void LDWS_GetResult_N(LDWS_Output **pLDWSOutput);

void LDWS_Get_inter_Pamer_W(int *Eu_W, int *Ev_W, int *Cx_W, int *Cy_W);

void LDWS_Get_inter_Pamer_N(int *Eu_N, int *Ev_N, int *Cx_N, int *Cy_N);

void LDWS_GetVanishPointSet(LDWS_Point *VanishPoint);

void LDWS_Finalization(void);

int LDWS_GetCarY(int y, double RealcarHeight);

void LDWS_GetXofY(int Y, int *x_left, int *x_right);

void LDWS_Get_Dist_xz(int x1, int y1, double *X, double *Z, double *detvanish);

double LDWS_GetXofWorld(int X, int Y);

double LDWS_GetXLengthofWorld(int imageXL, int Y);

double LDWS_GetXofImage(int X, int Y);

double LDWS_GetXLengthofImage(double worldXL, int Y);

int LDWS_GetFCWSD_th(void);

/***********************************************************************************************/

/*
 Function process:
 + malloc the memory for parameters
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
void LDWS_AllocModel(void) {
        Donnees = (Fichier *) malloc(sizeof(Fichier));
	memset(Donnees, 0, sizeof(Fichier));
        M_3D_init = (Modele_3D *) malloc(sizeof(Modele_3D));
	memset(M_3D_init, 0, sizeof(Modele_3D));
        M_3D_est = (Modele_3D *) malloc(sizeof(Modele_3D));
	memset(M_3D_est, 0, sizeof(Modele_3D));
        M_Filter = (Modele_Filter *) malloc(sizeof(Modele_Filter));
	memset(M_Filter, 0, sizeof(Modele_Filter));
        L_output = (LDWS_Output *) malloc(sizeof(LDWS_Output));
	memset(L_output, 0, sizeof(LDWS_Output));
}

/*
 Function process:
 + Init the values for parameters
 Fan-in :
 + main()
 Fan-out:
 + LDWS_InitRoadTracker()
 ATTENTION: __________
 */
void LDWS_Init(const char *fichier_init, const char *fichier_init_custom) {
	LDWS_InitRoadTracker(STATIQUE, &M_I_init, &M_I_est, M_3D_init, M_3D_est,
			M_Filter, L_output, Donnees, fichier_init, fichier_init_custom);
}

/*
 Function process:
 + The main realization of LDWS
 Fan-in :
 + main()
 Fan-out:
 + LDWS_RoadTracker()
 ATTENTION: __________
 */
void LDWS_Tracker(const unsigned char *Tab_Image) {
	LDWS_RoadTracker(&M_I_init, &M_I_est, M_3D_init, M_3D_est, M_Filter,
			L_output, Donnees, Tab_Image);
}

/*
 Function process:
 + free the model in LDWS
 Fan-in :
 + main()
 Fan-out:
 + Free_Modele_OutPut()
 + Free_Modele_Struct()
 + Free_Modele_Fichier()

 ATTENTION: __________
 */
void LDWS_Finalization(void) {
	Free_Modele_OutPut(L_output);

	Free_Modele_Struct(M_3D_init, M_3D_est, M_Filter);

	Free_Modele_Fichier(Donnees);
}

/*
 Function process:
 + Get the LDWS result
 Fan-in :
 + main()
 Fan-out:
 + LDWS_GetXYZofWorldfromImage()
 ATTENTION: Memory free in LDWS_FreeResult()
 */
void LDWS_GetResult(LDWS_Output **pLDWSOutput) {
	int i;
	//LDWS_3D_Point Point3D;

	if ((*pLDWSOutput) == NULL) {
                *pLDWSOutput = (LDWS_Output *) malloc(sizeof(LDWS_Output));
		memset(*pLDWSOutput, 0, sizeof(LDWS_Output));

                (*pLDWSOutput)->pPoint = (LDWS_Point *) malloc(
				L_output->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));
		memset((*pLDWSOutput)->pPoint, 0,
				L_output->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));
                (*pLDWSOutput)->p3DPoint = (LDWS_3D_Point *) malloc(
				L_output->LPointLength * LDWS_NB_BANDES
						* sizeof(LDWS_3D_Point));
		memset((*pLDWSOutput)->p3DPoint, 0,
				L_output->LPointLength * LDWS_NB_BANDES
						* sizeof(LDWS_3D_Point));
                (*pLDWSOutput)->pCaPoint = (LDWS_Point *) malloc(
				LDWS_NB_ZONES * sizeof(LDWS_Point));
		memset((*pLDWSOutput)->pCaPoint, 0, LDWS_NB_ZONES * sizeof(LDWS_Point));
                (*pLDWSOutput)->p3DCaPoint = (LDWS_3D_Point *) malloc(
				LDWS_NB_ZONES * sizeof(LDWS_3D_Point));
		memset((*pLDWSOutput)->p3DCaPoint, 0,
				LDWS_NB_ZONES * sizeof(LDWS_3D_Point));

                (*pLDWSOutput)->Param = (double *) malloc(
				LDWS_NB_PARAM * sizeof(double));
		memset((*pLDWSOutput)->Param, 0, LDWS_NB_PARAM * sizeof(double));

		(*pLDWSOutput)->NB_INTERVALLES = L_output->NB_INTERVALLES;
		(*pLDWSOutput)->NB_BANDES = L_output->NB_BANDES;

		for (i = 0; i < 6; i++) {
			(*pLDWSOutput)->Confidence_detection[i] =
					L_output->Confidence_detection[i];
		}
		(*pLDWSOutput)->LPointLength = L_output->LPointLength;
		(*pLDWSOutput)->Ev = L_output->Ev;
		(*pLDWSOutput)->Z0 = L_output->Z0;
		(*pLDWSOutput)->Tx = L_output->Tx;
		(*pLDWSOutput)->Ty = L_output->Ty;
	}

	if (L_output->Route == 1
			&& (L_output->Confidence_detection[0]
					> L_output->Confidence - KEEP_FRAME)) {
		(*pLDWSOutput)->Route = L_output->Route;
		(*pLDWSOutput)->Route_half = L_output->Route_half;
	} else if (L_output->Route == 1
			&& (L_output->Confidence_detection[0]
					<= L_output->Confidence - KEEP_FRAME)&&
			(L_output->Confidence_detection[1] > L_output->Confidence - KEEP_FRAME)){
			(*pLDWSOutput)->Route = L_output->Route;
			(*pLDWSOutput)->Route_half = 1;
		}
		else if(L_output->Route == 1 && (L_output->Confidence_detection[0] <= L_output->Confidence - KEEP_FRAME) &&
				(L_output->Confidence_detection[2] > L_output->Confidence - KEEP_FRAME))
		{
			(*pLDWSOutput)->Route = 2;
			(*pLDWSOutput)->Route_half = 0;
		}
		else if(L_output->Route == 1 && (L_output->Confidence_detection[0] <= L_output->Confidence - KEEP_FRAME) &&
				(L_output->Confidence_detection[3] > L_output->Confidence - KEEP_FRAME))
		{
			(*pLDWSOutput)->Route = 3;
			(*pLDWSOutput)->Route_half = 0;
		}
		else
		{
			(*pLDWSOutput)->Route = L_output->Route;
			(*pLDWSOutput)->Route_half = L_output->Route_half;
		}

	(*pLDWSOutput)->Route_L = L_output->Route_L;
	(*pLDWSOutput)->Route_R = L_output->Route_R;
	(*pLDWSOutput)->Confidence = L_output->Confidence;
	for (i = 0; i < 6; i++) {
		(*pLDWSOutput)->Confidence_detection[i] =
				L_output->Confidence_detection[i];
	}

	for (i = 0; i < LDWS_NB_PARAM; ++i) {
		(*pLDWSOutput)->Param[i] = L_output->Param[i];
	}

	for (i = 0; i < LDWS_NB_ZONES; ++i) {
		(*pLDWSOutput)->pCaPoint[i] = L_output->pCaPoint[i];

		//if(L_output->Route==1 && L_output->Confidence>Donnees->Confiance-KEEP_FRAME)
		{
			//LDWS_GetXYZofWorldfromImage(L_output->pCaPoint[i].x, L_output->pCaPoint[i].y, Donnees->Cx, Donnees->Cy,
			//	                        Donnees->Z0, Donnees->Ev,L_output->Param[3], &Point3D.X, &Point3D.Y, &Point3D.Z);

			//(*pLDWSOutput)->p3DCaPoint[i]=Point3D;
		}
	}

	//for (i = 0; i < L_output->LPointLength * LDWS_NB_BANDES; ++i)
	//{
	//	(*pLDWSOutput)->pPoint[i] = L_output->pPoint[i];
	//	//if(L_output->Route==1)
	//	{
	//	LDWS_GetXYZofWorldfromImage(L_output->pPoint[i].x, L_output->pPoint[i].y, Donnees->Cx, Donnees->Cy,
	//		                        Donnees->Z0, Donnees->Ev,L_output->Param[3], &Point3D.X, &Point3D.Y, &Point3D.Z);

	//	(*pLDWSOutput)->p3DPoint[i]=Point3D;

	//	}
	//}

	(*pLDWSOutput)->alarm_result = AlarmMain(*pLDWSOutput);

}

/*
 Function process:
 + Free the LDWS result
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void LDWS_FreeResult(LDWS_Output **pLDWSOutput)
{
        free((*pLDWSOutput)->pPoint);
	(*pLDWSOutput)->pPoint = NULL;

        free((*pLDWSOutput)->p3DPoint);
	(*pLDWSOutput)->p3DPoint = NULL;

        free((*pLDWSOutput)->pCaPoint);
	(*pLDWSOutput)->pCaPoint = NULL;

        free((*pLDWSOutput)->p3DCaPoint);
	(*pLDWSOutput)->p3DCaPoint = NULL;

        free((*pLDWSOutput)->Param);
	(*pLDWSOutput)->Param = NULL;

        free(*pLDWSOutput);
	*pLDWSOutput = NULL;

}

/*
 Function process:
 + return Donnees->runFlag
 */
int LDWS_Get_RunFlag(void) {
	return (Donnees->runFlag);
}

/*
 Function process:
 + return Donnees->carWidth
 */
double LDWS_GetCarWidth(void) {
	return (Donnees->carWidth);
}

/*
Function process:
+ return Donnees->Z0
*/
double LDWS_GetCameraHeight(void) {
	return (Donnees->Z0);
}

/*
 Function process:
 + return Donnees->leftDeviation
 */
double LDWS_GetLeftDeviation(void) {
	return (Donnees->leftDeviation);
}

/*
 Function process:
 + return Donnees->rightDeviation
 */
double LDWS_GetRightDeviation(void) {
	return (Donnees->rightDeviation);
}

/*
 Function process:
 + return Donnees->TTCWaring
 */
double LDWS_GetWarningTTC(void) {
	return (Donnees->TTCWaring);
}

/*
 Function process:
 + change the wide result to the narrrow result
 Fan-in :
 + main()
 Fan-out:
 + LDWS_GetxyofImagefromWorld()
 ATTENTION: __________
 */
void LDWS_ChangeResultWtoN(void) {
	/*
	int i;
	LDWS_Point Point2D;

	LDWS_GetResult(&L_output_N);

	if (LDWS_Get_RunFlag()) {
		for (i = 0; i < LDWS_NB_ZONES; ++i) {
			//if((L_output_N->Route==1) && (L_output_N->Confidence_detection)>(L_output_N->Confidence-KEEP_FRAME))
			{
				LDWS_GetxyofImagefromWorld(&Point2D.x, &Point2D.y,
						Donnees->Cx_N, Donnees->Cy_N, Donnees->Z0,
						Donnees->Ev_N, L_output_N->Param[3],
						L_output_N->p3DCaPoint[i].X + 0.08,
						L_output_N->p3DCaPoint[i].Y,
						L_output_N->p3DCaPoint[i].Z);
				L_output_N->pCaPoint[i] = Point2D;
			}
		}

		for (i = 0; i < L_output->LPointLength * LDWS_NB_BANDES; ++i) {
			//if((L_output_N->Route==1) && (L_output_N->Confidence_detection)>(L_output_N->Confidence-KEEP_FRAME))
			{
				LDWS_GetxyofImagefromWorld(&Point2D.x, &Point2D.y,
						Donnees->Cx_N, Donnees->Cy_N, Donnees->Z0,
						Donnees->Ev_N, L_output_N->Param[3],
						L_output_N->p3DPoint[i].X + 0.08,
						L_output_N->p3DPoint[i].Y, L_output_N->p3DPoint[i].Z);
				L_output_N->pPoint[i] = Point2D;
			}
		}

		L_output_N->Ev = Donnees->Ev_N;
	}
	*/
}

/*
 Function process:
 + Get the coord of 3D world (X,Y,Z) basd on the ground point (x,y)
 Fan-in :
 + LDWS_GetResult()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
void LDWS_GetXYZofWorldfromImage(int img_x, int img_y, const int Cx,
		const int Cy, const double H, const int f, const double alpha,
		double *world_X, double *world_Y, double *world_Z) {
/*
	img_x -= Cx;
	img_y = Cy - img_y;
	*world_Y = -H;
	*world_X = (*world_Y) * img_x / (img_y * cos(alpha) - f * sin(alpha));
	*world_Z = (*world_Y) * (f * cos(alpha) + img_y * sin(alpha))
			/ (img_y * cos(alpha) - f * sin(alpha));
*/
}

/*
 Function process:
 + Get the coord of image ground point (x,y) basd on the3D world (X,Y,Z)
 Fan-in :
 + LDWS_ChangeResultWtoN()
 Fan-out:
 + N/A
 ATTENTION: __________
 */
void LDWS_GetxyofImagefromWorld(int *img_x, int *img_y, int Cx, int Cy,
		double H, int f, double alpha, double world_X, double world_Y,
		double world_Z) {
			/*
	*img_x = (int) ((f * world_X)
			/ (world_Z * cos(alpha) - world_Y * sin(alpha)) + 0.5);
	*img_y = (int) ((f * (world_Y * cos(alpha) + world_Z * sin(alpha))
			/ (world_Z * cos(alpha) - world_Y * sin(alpha))) + 0.5);
	*img_x += Cx;
	*img_y = Cy - *img_y;
	*/
}

/*
 Function process:
 + Get the Init LDWS detect zone
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION: Memory free in LDWS_Freeinit()
 */
void LDWS_Getinit(LDWS_InitGuid **pLDWSInit) {
	int i;

	if ((*pLDWSInit) == NULL) {
                *pLDWSInit = (LDWS_InitGuid *) malloc(sizeof(LDWS_InitGuid));
		memset(*pLDWSInit, 0, sizeof(LDWS_InitGuid));
		
		(*pLDWSInit)->pCaPoint	= (LDWS_Point *) malloc(LDWS_NB_ZONES * sizeof(LDWS_Point));
		memset((*pLDWSInit)->pCaPoint, 0, LDWS_NB_ZONES * sizeof(LDWS_Point));

                (*pLDWSInit)->pBoundPoint = (LDWS_Point *) malloc(
				7 * sizeof(LDWS_Point));
		memset((*pLDWSInit)->pBoundPoint, 0, 7 * sizeof(LDWS_Point));

                (*pLDWSInit)->Param = (double *)malloc(
			LDWS_NB_PARAM * sizeof(double));
		memset((*pLDWSInit)->Param, 0, LDWS_NB_PARAM * sizeof(double));

		(*pLDWSInit)->NB_INTERVALLES = Donnees->NB_INTERVALLES;
		(*pLDWSInit)->NB_BANDES = LDWS_NB_BANDES;
	}

	for (i = 0; i < LDWS_NB_ZONES; ++i)
	{
		(*pLDWSInit)->pCaPoint[i].x = (int)(M_3D_init->X[i] + 0.5);
		(*pLDWSInit)->pCaPoint[i].y = (int)(M_I_init.V[i] + 0.5);
	}
	i = 0;
	(*pLDWSInit)->pBoundPoint[i].x = (int) (M_3D_init->X[0]);    //P1
	(*pLDWSInit)->pBoundPoint[i].y = (int) (M_I_init.V[0]);

	i++;
	(*pLDWSInit)->pBoundPoint[i].x =
			(int) (M_3D_init->X[Donnees->NB_INTERVALLES]);    //P2
	(*pLDWSInit)->pBoundPoint[i].y =
			(int) (M_I_init.V[Donnees->NB_INTERVALLES]);

	i++;
	(*pLDWSInit)->pBoundPoint[i].x = (int) (M_3D_init->X[2
			* Donnees->NB_INTERVALLES - 1]);    //P3
	(*pLDWSInit)->pBoundPoint[i].y = (int) (M_I_init.V[2
			* Donnees->NB_INTERVALLES - 1]);

	i++;
	(*pLDWSInit)->pBoundPoint[i].x = (int) (M_3D_init->X[Donnees->NB_INTERVALLES
			- 1]);    //P4
	(*pLDWSInit)->pBoundPoint[i].y = (int) (M_I_init.V[Donnees->NB_INTERVALLES
			- 1]);

	i++;
	(*pLDWSInit)->pBoundPoint[i].x = (int) (M_3D_init->X[0]);    //P1
	(*pLDWSInit)->pBoundPoint[i].y = (int) (M_I_init.V[0]);

	i++;
	(*pLDWSInit)->pBoundPoint[i].x =
			(int) (0.5
					* ((*pLDWSInit)->pBoundPoint[0].x
							+ (*pLDWSInit)->pBoundPoint[1].x));    //M1
	(*pLDWSInit)->pBoundPoint[i].y =
			(int) (0.5
					* ((*pLDWSInit)->pBoundPoint[0].y
							+ (*pLDWSInit)->pBoundPoint[1].y));

	i++;
	(*pLDWSInit)->pBoundPoint[i].x =
			(int) (0.5
					* ((*pLDWSInit)->pBoundPoint[2].x
							+ (*pLDWSInit)->pBoundPoint[3].x));    //M2
	(*pLDWSInit)->pBoundPoint[i].y =
			(int) (0.5
					* ((*pLDWSInit)->pBoundPoint[2].y
							+ (*pLDWSInit)->pBoundPoint[3].y));

	for (i = 0; i < LDWS_NB_PARAM; ++i) {
		(*pLDWSInit)->Param[i] = M_3D_init->Param[i];
	}
}

/*
 Function process:
 + Free the Init LDWS detect zone
 Fan-in :
 + main()
 Fan-out:
 + N/A
 ATTENTION:
 */
void LDWS_Freeinit(LDWS_InitGuid **pLDWSInit) {
        free((*pLDWSInit)->pCaPoint);
	(*pLDWSInit)->pCaPoint = NULL;

        free((*pLDWSInit)->pBoundPoint);
	(*pLDWSInit)->pBoundPoint = NULL;

	if ((*pLDWSInit)->Param != NULL)
	{
                free((*pLDWSInit)->Param);
		(*pLDWSInit)->Param = NULL;
	}

        free((*pLDWSInit));
	(*pLDWSInit) = NULL;
}

/*
 Function process:
 + return Donnees->Vh
 */
int LDWS_GetVanishY(void) {
	return (Donnees->vanishPoint.y);
}

/*
 Function process:
 + get vanishY line of narrow
 */
int LDWS_GetVanishY_N(void) {
	return (Donnees->Cy_N
			- (Donnees->Cy - Donnees->Vh) * Donnees->Ev_N / Donnees->Ev);
}

/*
 Function process:
 + Get the LDWS result on narrow
 Fan-in :
 + main()
 Fan-out:
 +
 ATTENTION: Memory free in LDWS_FreeResult()
 */
void LDWS_GetResult_N(LDWS_Output **pLDWSOutput) {
	/*
	int i;

	if ((*pLDWSOutput) == NULL) {
                *pLDWSOutput = (LDWS_Output *) malloc(sizeof(LDWS_Output));
		memset(*pLDWSOutput, 0, sizeof(LDWS_Output));

                (*pLDWSOutput)->pPoint = (LDWS_Point *) malloc(
				L_output_N->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));
		memset((*pLDWSOutput)->pPoint, 0,
				L_output_N->LPointLength * LDWS_NB_BANDES * sizeof(LDWS_Point));
                (*pLDWSOutput)->p3DPoint = (LDWS_3D_Point *) malloc(
				L_output_N->LPointLength * LDWS_NB_BANDES
						* sizeof(LDWS_3D_Point));
		memset((*pLDWSOutput)->p3DPoint, 0,
				L_output_N->LPointLength * LDWS_NB_BANDES
						* sizeof(LDWS_3D_Point));
                (*pLDWSOutput)->pCaPoint = (LDWS_Point *) malloc(
				LDWS_NB_ZONES * sizeof(LDWS_Point));
		memset((*pLDWSOutput)->pCaPoint, 0, LDWS_NB_ZONES * sizeof(LDWS_Point));
                (*pLDWSOutput)->p3DCaPoint = (LDWS_3D_Point *) malloc(
				LDWS_NB_ZONES * sizeof(LDWS_3D_Point));
		memset((*pLDWSOutput)->p3DCaPoint, 0,
				LDWS_NB_ZONES * sizeof(LDWS_3D_Point));

                (*pLDWSOutput)->Param = (double *) malloc(
				LDWS_NB_PARAM * sizeof(double));
		memset((*pLDWSOutput)->Param, 0, LDWS_NB_PARAM * sizeof(double));

		(*pLDWSOutput)->NB_INTERVALLES = L_output_N->NB_INTERVALLES;
		(*pLDWSOutput)->NB_BANDES = L_output_N->NB_BANDES;

		for (i = 0; i < 6; i++) {
			(*pLDWSOutput)->Confidence_detection[i] =
					L_output_N->Confidence_detection[i];
		}
		(*pLDWSOutput)->LPointLength = L_output_N->LPointLength;
		(*pLDWSOutput)->Ev = L_output_N->Ev;
		(*pLDWSOutput)->Z0 = L_output_N->Z0;
		(*pLDWSOutput)->Tx = L_output_N->Tx;
		(*pLDWSOutput)->Ty = L_output_N->Ty;
	}
	(*pLDWSOutput)->Route = L_output_N->Route;
	(*pLDWSOutput)->Route_L = L_output_N->Route_L;
	(*pLDWSOutput)->Route_R = L_output_N->Route_R;
	(*pLDWSOutput)->Confidence = L_output_N->Confidence;
	for (i = 0; i < 6; i++) {
		(*pLDWSOutput)->Confidence_detection[i] =
				L_output_N->Confidence_detection[i];
	}

	for (i = 0; i < LDWS_NB_PARAM; ++i) {
		(*pLDWSOutput)->Param[i] = L_output_N->Param[i];
	}

	for (i = 0; i < LDWS_NB_ZONES; ++i) {
		(*pLDWSOutput)->pCaPoint[i] = L_output_N->pCaPoint[i];
		//if(L_output_N->Route==1 && L_output_N->Confidence>Donnees->Confiance-KEEP_FRAME)
		{
			(*pLDWSOutput)->p3DCaPoint[i] = L_output_N->p3DCaPoint[i];
		}
	}

	for (i = 0; i < L_output_N->LPointLength * LDWS_NB_BANDES; ++i) {
		(*pLDWSOutput)->pPoint[i] = L_output_N->pPoint[i];
		//if(L_output_N->Route==1)
		{
			(*pLDWSOutput)->p3DPoint[i] = L_output_N->p3DPoint[i];
		}
	}
	*/
}

/*
 Function process:
 + get inter param on width
 */
void LDWS_Get_inter_Pamer_W(int *Eu_W, int *Ev_W, int *Cx_W, int *Cy_W) {
	*Eu_W = Donnees->Eu;
	*Ev_W = Donnees->Ev;
	*Cx_W = Donnees->Cx;
	*Cy_W = Donnees->Cy;
}

/*
 Function process:
 + get inter param on narrow
 */
void LDWS_Get_inter_Pamer_N(int *Eu_N, int *Ev_N, int *Cx_N, int *Cy_N) {
	*Eu_N = Donnees->Eu_N;
	*Ev_N = Donnees->Ev_N;
	*Cx_N = Donnees->Cx_N;
	*Cy_N = Donnees->Cy_N;
}

/*
 Function process:
 + Calcualte the statistic vanish point on detected window
 */
void LDWS_CalVanishPointSet(void) {
	static int max;
	static int intflag = 0;
	int i;
	int x, y;
	int index[9] = {0};
	int index_x[9] = {0, 0, -4, 4, 0, -4, 4, -4, 4};
	int index_y[9] = {0, -4, 0, 0, 4, -4, -4, 4, 4};

	if (intflag == 0) {
                pLDWSVPoint.hist = (int *) malloc(
				(L_output->Tx >> 2) * (L_output->Ty >> 2) * sizeof(int));
		memset(pLDWSVPoint.hist, 0,
				(L_output->Tx >> 2) * (L_output->Ty >> 2) * sizeof(int));

		pLDWSVPoint.max = 0;
		pLDWSVPoint.maxVPoint.x = INVAILD_TAN;
		pLDWSVPoint.maxVPoint.y = INVAILD_TAN;
		intflag =1;
	}

	if ((L_output != NULL) && (L_output->Route == 1)
			&& (L_output->Confidence_detection[0]
					> L_output->Confidence - KEEP_FRAME)) {
		double a1 = (L_output->pCaPoint[5].y - L_output->pCaPoint[3].y)
				/ (L_output->pCaPoint[5].x - L_output->pCaPoint[3].x
						+ 0.00001);
		double b1 = L_output->pCaPoint[5].y
				- a1 * (L_output->pCaPoint[5].x);

		double a2 = (L_output->pCaPoint[5 + L_output->NB_INTERVALLES].y
				- L_output->pCaPoint[3 + L_output->NB_INTERVALLES].y)
				/ (L_output->pCaPoint[5 + L_output->NB_INTERVALLES].x
						- L_output->pCaPoint[3 + L_output->NB_INTERVALLES].x
						+ 0.00001);
		double b2 = L_output->pCaPoint[5 + L_output->NB_INTERVALLES].y
				- a2 * (L_output->pCaPoint[5 + L_output->NB_INTERVALLES].x);

		x = (int) ((b1 - b2) / (a2 - a1 + 0.00001) + 0.5);
		y = (int) (a1 * x + b1 + 0.5);

		if (x >= 2 && x < L_output->Tx-2 && y >= 2 && y < L_output->Ty-2) 
		{
			index[0] = (int)((y>>2)*(L_output->Tx>>2) + (x >> 2));
			index[1] = index[0] - L_output->Tx>>2;
			index[5] = index[1] - 1;
			index[6] = index[1] + 1;
			index[2] = index[0] - 1;
			index[3] = index[0] + 1;
			index[4] = index[0] + L_output->Tx>>2;
			index[7] = index[4] - 1;
			index[8] = index[4] + 1;
			for (i = 0; i < 8; i++)
			{
				if (++(pLDWSVPoint.hist[index[i]])>= max) 
				{
					max = pLDWSVPoint.hist[index[i]];
					pLDWSVPoint.maxVPoint.x = x + index_x[i];
					pLDWSVPoint.maxVPoint.y = y + index_y[i];
					pLDWSVPoint.max = max;
				}
			}		
			//printf("x:%d,y:%d,max:%d\n",pLDWSVPoint.maxVPoint.x,pLDWSVPoint.maxVPoint.y,max);
		}
	}

	if(max > 150)
	{
		memset(pLDWSVPoint.hist, 0,
				(L_output->Tx >> 2) * (L_output->Ty >> 2) * sizeof(int));
		max = 0;
		pLDWSVPoint.max = 0;
		pLDWSVPoint.hist[(pLDWSVPoint.maxVPoint.y>>2)*(L_output->Tx>>2) + (pLDWSVPoint.maxVPoint.x >> 2)] = 50;
	}

    if(pLDWSVPoint.max > 0)
	{
		Donnees->vanishPoint.x = pLDWSVPoint.maxVPoint.x;
		Donnees->vanishPoint.y = pLDWSVPoint.maxVPoint.y;
	}
	else
	{
		Donnees->vanishPoint.x = Donnees->Cx_N + Donnees->Eu * Donnees->Psi * A_TO_R;
		Donnees->vanishPoint.y = Donnees->Cy_N - Donnees->Ev * Donnees->Alpha * A_TO_R;
	}

	Donnees->Vangle = atan((double)(Donnees->Cy_N - Donnees->vanishPoint.y)/Donnees->Ev_N);

}

/*
 Function process:
 + Get the statistic vanish point on detected window
 */
void LDWS_GetVanishPointSet(LDWS_Point *VanishPoint) {
	VanishPoint->x = Donnees->vanishPoint.x;
	VanishPoint->y = Donnees->vanishPoint.y;
}

/*
 Function process:
 + Get the image car height based on the y image ground point and real car height of 3D world
 Fan-in :
 + main()
 Fan-out:
 + N/A
 */
int LDWS_GetCarY(int y, double RealcarHeight) {
	/*
	if (L_output != NULL) {
		double Y1, cos_theat, sin_theat, Z1, Y2, y2;
		y = Donnees->Cy_N - y;
		Y1 = -L_output->Z0;
		cos_theat = cos(L_output->Param[3]);
		sin_theat = sin(L_output->Param[3]);

		Z1 = Y1 * (Donnees->Ev_N * cos_theat + y * sin_theat)
				/ (y * cos_theat - Donnees->Ev_N * sin_theat);
		Y2 = Y1 + RealcarHeight;
		y2 = Donnees->Ev_N * (Y2 * cos_theat + Z1 * sin_theat)
				/ (Z1 * cos_theat - Y2 * sin_theat);

		return (int) (y2 - y + 0.5);
	} else {
		return (-100000);
	}*/
	if (L_output != NULL) 
	{
		double Y1, cos_theat, sin_theat, Z1, Y2, y2;
		y = Donnees->Cy_N - y;
		Y1 = -Donnees->Z0;
		cos_theat = cos(Donnees->Vangle);
		sin_theat = sin(Donnees->Vangle);

		Z1 = Y1 * (Donnees->Ev_N * cos_theat + y * sin_theat)
				/ (y * cos_theat - Donnees->Ev_N * sin_theat);
		Y2 = Y1 + RealcarHeight;
		y2 = Donnees->Ev_N * (Y2 * cos_theat + Z1 * sin_theat)
				/ (Z1 * cos_theat - Y2 * sin_theat);

		return (int) (y2 - y + 0.5);
	}
	else 
	{
		return (-100000);
	}
}

/*
 Function process:
 + Get the car height of 3D world based on the y1 image ground point and y2 image top point
 Fan-in :
 + main()
 Fan-out:
 + N/A
 */
double LDWS_GetImageY(int y1, int y2) {
	if (L_output != NULL) {
		double Y1, cos_theat, sin_theat, Z1, Y2;
		y1 = Donnees->Cy_N - y1;
		y2 = Donnees->Cy_N - y2;
		Y1 = -L_output->Z0;
		cos_theat = cos(Donnees->Vangle);
		sin_theat = sin(Donnees->Vangle);

		Z1 = Y1 * (Donnees->Ev_N * cos_theat + y1 * sin_theat)
				/ (y1 * cos_theat - Donnees->Ev_N * sin_theat);
		Y2 = (Z1 * (y2 * cos_theat - Donnees->Ev_N * sin_theat)
				/ (Donnees->Ev_N * cos_theat + y1 * sin_theat));

		return (Y2 - Y1);
	} else {
		return (-100000);
	}
}

/*
 Function process:
 + Get the left and right intersection point based on the y coord of ground point with the LDWS line
 Fan-in :
 + main()
 Fan-out:
 + N/A
 */
void LDWS_GetXofY(int Y, int *x_left, int *x_right) {
	int i;
	double tempK;
	*x_left = INVAILD_TAN;
	*x_right = INVAILD_TAN;

	if (L_output != NULL)    // && L_output_N->Route == 1)
			{
		for (i = 0; i < L_output->NB_INTERVALLES - 1; ++i) {
			if (Y >= L_output->pCaPoint[i].y
					&& Y <= L_output->pCaPoint[i + 1].y) {
				tempK = (double) (L_output->pCaPoint[i + 1].y - Y)
						/ (L_output->pCaPoint[i + 1].y
								- L_output->pCaPoint[i].y);
				*x_left = L_output->pCaPoint[i + 1].x
						+ tempK
								* (L_output->pCaPoint[i].x
										- L_output->pCaPoint[i + 1].x);
				*x_right =
						L_output->pCaPoint[i + 1 + L_output->NB_INTERVALLES].x
								+ tempK
										* (L_output->pCaPoint[i
												+ L_output->NB_INTERVALLES].x
												- L_output->pCaPoint[i + 1
														+ L_output->NB_INTERVALLES].x);
			}
		}
	}
}

/*
 Function process:
 + Get the X and Z of 3D world and vanish line based on (x1,y1)ground point of image
 Fan-in :
 + main()
 Fan-out:
 + N/A
 */
void LDWS_Get_Dist_xz(int x1, int y1, double *X, double *Z, double *detvanish) {
	/*
	double Dist_X, Dist_Z;
	x1 -= Donnees->Cx_N;
	y1 = Donnees->Cy_N - y1;
	Dist_Z = -Donnees->Z0
			* (Donnees->Ev_N * cos(L_output->Param[3])
					+ y1 * sin(L_output->Param[3]))
			/ (y1 * cos(L_output->Param[3])
					- Donnees->Ev_N * sin(L_output->Param[3]));
	if (Dist_Z < 0)
		*Z = 0;
	else
		*Z = Dist_Z;
	Dist_X = -Donnees->Z0 * (x1)
			/ (y1 * cos(L_output->Param[3])
					- Donnees->Ev_N * sin(L_output->Param[3]));
	*X = Dist_X;
	*detvanish = Donnees->Cy_N - Donnees->Ev_N * tan(L_output->Param[3]);
	*/
	x1 -= Donnees->Cx_N;
	y1 = Donnees->Cy_N - y1;
	*X = (- x1 * Donnees->Z0) / (y1*cos(Donnees->Vangle)-Donnees->Ev_N*sin(Donnees->Vangle));
	*Z = -Donnees->Z0 * (Donnees->Ev_N * cos(Donnees->Vangle) + y1 * sin(Donnees->Vangle))
			/ (y1 * cos(Donnees->Vangle) - Donnees->Ev_N * sin(Donnees->Vangle));
	*detvanish = Donnees->vanishPoint.y;

}

/*
 Function process:
 + Given the image y and line width in pixels and get the word line width
 */
double LDWS_GetDetaXofWorld(int imagexL, int y) {
	//double cos_theat, sin_theat;

	//y = Donnees->Cy_N-y;
	//cos_theat	= cos(L_output->Param[3]);
	//sin_theat	= sin(L_output->Param[3]);

	//printf("LDWS:%f, FCWS:%f\n",imagexL*Donnees->Z0/(y-Donnees->Vh),-Donnees->Z0 * imagexL / (y * cos_theat - Donnees->Ev_N * sin_theat));
	//return (-Donnees->Z0 * imagexL / (y * cos_theat - Donnees->Ev_N * sin_theat));//FCWSD
	//printf("imagexL:%d, y:%d,LDWS:%f\n",imagexL,y,imagexL*Donnees->Z0/(y-Donnees->Vh));

	return (imagexL*Donnees->Z0/(y-Donnees->Vh));//LDWS
}

/*
 Function process:
 + Given the point of image and get the X distance of world
 Fan-in :
 + main()
 Fan-out:
 + LDWS_GetXLengthofWorld()
 ATTENTION: __________
 */
double LDWS_GetXofWorld(int X, int Y) {
	X -= Donnees->Cx_N;

	return LDWS_GetXLengthofWorld(X, Y);
}

double LDWS_GetXofWorld_W(int X, int Y) {
	X -= Donnees->Cx;
	return (X / (Donnees->kWidth * (Y - Donnees->Vh)));
}

double LDWS_GetXLengthofWorld(int imageXL, int Y)
{
	//double debugValue = (double)(Donnees->Cy_N + Donnees->Ev_N*tan(-L_output->Param[3]));
	//double Vangle = atan((double)(VanishY - Donnees->Cy_N)/Donnees->Ev_N);
	//printf("Y,V: %d,%f\n",Y,debugValue);
	if(Y <= Donnees->vanishPoint.y )
		return -10000;
	else
	    return ((-imageXL * Donnees->Z0) / ((Donnees->Cy_N -Y)*cos(Donnees->Vangle)-Donnees->Ev_N*sin(Donnees->Vangle)));
}

//double LDWS_GetXofImage(int X, int Y) {
//	return (LDWS_GetXLengthofWorld(X, Y) + Donnees->Cx_N);
//}

/*
Function process:
	+ Get the Deta_x1 (pixels) of image based on Deta_X1 (m) in 3D world and Y on ground point of image
*/
double LDWS_GetXLengthofImage(double worldXL, int Y)
{
	//return ((worldXL *  ((Y - Donnees->Cy_N)*cos(L_output->Param[3])-Donnees->Ev_N*sin(L_output->Param[3])))/Donnees->Z0);
	return ( worldXL * (Y - Donnees->Vh)/Donnees->Z0);
}

int LDWS_GetFCWSD_th(void)
{
	return Donnees->FCWSD_th;
}

double LDWS_GetVehiclePosX(int x_img, int y_img)
{
	int v = 0;
	double v_evalpha = 0, div = 0, revalue = 0;

	v = Donnees->Cy - y_img;

	if (L_output != NULL && L_output->Route == 1)
	{
		v_evalpha = v - (double)(Donnees->Ev * L_output->Param[3]);
		if(v_evalpha<0)
		{
			div = v_evalpha / (double)(-Donnees->Ev * Donnees->Z0);
			revalue = ((x_img - Donnees->Cx) / (Donnees->Eu * div) - 
						L_output->Param[2] / div - L_output->Param[4] / (2 * div * div) - 
						L_output->Param[5] / (6 * div * div * div));
		}
		else
			return(-10000);
	}
	else
	{
		v_evalpha = v - (double)(Donnees->Ev * M_3D_init->Param[3]);
		if(v_evalpha<0)
		{
			div = v_evalpha / (double)(-Donnees->Ev * Donnees->Z0);
			revalue = ((x_img - Donnees->Cx) / (Donnees->Eu * div) - 
						M_3D_init->Param[2] / div - M_3D_init->Param[4] / (2 * div * div) - 
						M_3D_init->Param[5] / (6 * div * div * div));
		}
		else
			return(-10000);

	}

	return(revalue);

}
