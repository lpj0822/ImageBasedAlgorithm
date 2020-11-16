#ifndef _LDWS_INTERFACE_H_
#define _LDWS_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define KEEP_FRAME              (3)

//#define _LDWS_init_guid

typedef struct 
{
	int x;
	int y;
} LDWS_Point;

typedef struct 
{
	double X;
	double Y;
	double Z;
} LDWS_3D_Point;

typedef struct
{
	LDWS_Point	maxVPoint;
	int			*hist;
	int			max;
} LDWS_VPoint;

typedef struct 
{
	double Z0;

	double *Param;
	LDWS_Point *pPoint;
	LDWS_Point *pCaPoint;
	LDWS_3D_Point *p3DPoint;
	LDWS_3D_Point *p3DCaPoint;
	int Route;
	int Route_half;
	int Route_L;
	int Route_R;
	int NB_INTERVALLES;
	int NB_BANDES;
	int Confidence_detection[6];
	int LPointLength;
	int Confidence;
	int Ev;
	int Tx;
	int Ty;

	int alarm_result;
} LDWS_Output;

typedef struct 
{
	double *Param;
	LDWS_Point *pCaPoint;
	LDWS_Point *pBoundPoint;//P1,P2,P3,P4,P1,M1,M2
	int NB_INTERVALLES;
	int NB_BANDES;	
} LDWS_InitGuid;

/*
Realized function:
    + malloc the memory for parameters
*/
extern void LDWS_AllocModel(void);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    fichier_init	      const char*		  Config files to Init paramters.
[in]	    fichier_init_custum	  const char*		  Customer config files to Init paramters.

Realized function:
    + Init the values for parameters
*/
extern void LDWS_Init(const char *fichier_init,const char *fichier_init_custom);

/*
I/O:	    Name		          Type	     		          Content
					    						  
[in]	    Tab_Image	          const unsigned char*		  Input image buffer.

Realized function:
    + The main realization of LDWS
*/
extern void LDWS_Tracker(const unsigned char *Tab_Image);

/*
Realized function:
    + free the model in LDWS
*/
extern void LDWS_Finalization(void);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	L_output		      LDWS_Output**	      Output parameters model.

Realized function:
    + Get the LDWS result
*/
extern void LDWS_GetResult(LDWS_Output **pLDWSOutput);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in]	    img_x		      int		     	  x coord of ground point in image.
[in]	    img_y		      int		     	  y coord of ground point in image.
[in]	    Cx		          const int		      x coord of center point of image.
[in]	    Cy		          const int		      y coord of center point of image.
[in]	    H		          const int		      height of camera.
[in]	    f		          const int		      focal of camera.
[in]	    alpha		      const double		  pitch angle.
[in/out]	world_X		      double*	          The X coord in 3D world.
[in/out]	world_Y		      double*	          The Y coord in 3D world.
[in/out]	world_Z		      double*	          The Z coord in 3D world.

Realized function:
    + Get the coord of 3D world (X,Y,Z) basd on the ground point (x,y)
*/
extern void LDWS_GetXYZofWorldfromImage(int img_x, int img_y, const int Cx, const int Cy, const double H, const int f,
								        const double alpha, double *world_X, double *world_Y, double *world_Z);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	L_output		      LDWS_Output**	      Output parameters model.

Realized function:
    + Free pLDWSOutput
*/
extern void LDWS_FreeResult(LDWS_Output **pLDWSOutput);

/*
Realized function:
    + get Donnees->runFlag
*/
extern int LDWS_Get_RunFlag(void);

/*
Realized function:
    + get Donnees->carWidth
*/
extern double LDWS_GetCarWidth(void);

/*
Realized function:
+ get Donnees->Z0
*/
extern double LDWS_GetCameraHeight(void);

/*
Realized function:
    + get Donnees->leftDeviation
*/
extern double LDWS_GetLeftDeviation(void);

/*
Realized function:
    + get Donnees->rightDeviation
*/
extern double LDWS_GetRightDeviation(void);

/*
Realized function:
    + get Donnees->TTCWaring
*/
extern double LDWS_GetWarningTTC(void);

/*
Realized function:
    + given the value of L_output_N
*/
extern void LDWS_ChangeResultWtoN(void);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in/out]	img_x		      int		     	  x coord of ground point in image.
[in/out]	img_y		      int		     	  y coord of ground point in image.
[in]	    Cx		          const int		      x coord of center point of image.
[in]	    Cy		          const int		      y coord of center point of image.
[in]	    H		          const int		      height of camera.
[in]	    f		          const int		      focal of camera.
[in]	    alpha		      const double		  pitch angle.
[in]	    world_X		      double*	          The X coord in 3D world.
[in]	    world_Y		      double*	          The Y coord in 3D world.
[in]	    world_Z		      double*	          The Z coord in 3D world.

Realized function:
    + Get the coord of image ground point (x,y) basd on the3D world (X,Y,Z) 
*/
extern void	LDWS_GetxyofImagefromWorld(int *img_x, int *img_y, int Cx, int Cy, double H, int f,double alpha, double world_X, double world_Y, double world_Z);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pLDWSInit		      LDWS_InitGuid**	  Output Init parameters.

Realized function:
    + Get the Init LDWS detected zone
*/
extern void LDWS_Getinit(LDWS_InitGuid **pLDWSInit);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	pLDWSInit		      LDWS_InitGuid**	  Output Init parameters.

Realized function:
    + Free the Init LDWS detected zone
*/
extern void LDWS_Freeinit(LDWS_InitGuid **pLDWSInit);

/*
Realized function:
    + get Donnees->Vh
*/
extern int LDWS_GetVanishY(void);

/*
Realized function:
    + get vanishY line of narrow
*/
extern int LDWS_GetVanishY_N(void);

/*
Realized function:
    + get the LDWS result on narrow
*/
extern void LDWS_GetResult_N(LDWS_Output **pLDWSOutput);

/*
Realized function:
    + get inter param on wide camera
*/
extern void LDWS_Get_inter_Pamer_W(int *Eu_W, int *Ev_W, int *Cx_W, int *Cy_W);

/*
Realized function:
    + get inter param on narrow camera
*/
extern void LDWS_Get_inter_Pamer_N(int *Eu_N, int *Ev_N, int *Cx_N, int *Cy_N);


extern void LDWS_CalVanishPointSet(void);
/*
Realized function:
    + get the statistic vanish point on detected window
*/
extern void LDWS_GetVanishPointSet(LDWS_Point *VanishPoint);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in]	    y		          int		     	  y coord of ground point of image.
[in]	    RealcarHeight	  double		      real car height (m).

[out]	    returned		  int	              The image car height on image.

Realized function:
    + Get the image car height based on the y image ground point and real car height of 3D world
*/
extern int LDWS_GetCarY(int y, double RealcarHeight);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in]	    y1		          int		     	  y coord of ground point of image.
[in]	    y2	              int		          y coord of top point of image.

[out]	    returned		  double	          The caculate RealcarHeight.

Realized function:
    + Get the car height of 3D world based on the y1 image ground point and y2 image top point
*/
extern double LDWS_GetImageY(int y1, int y2);

//extern double LDWS_GetImageYTrack(int y1, int y2);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in]	    y		          int		     	  y coord of ground point of image.
[in/out]	x_left	          int*		          left x coord intersection point between y and LDWS line.
[in/out]	x_right	          int*		          right x coord intersection point between y and LDWS line.

Realized function:
    + Get the left and rightintersection point based on the y coord of ground point with the LDWS line
*/
extern void LDWS_GetXofY(int Y, int *x_left, int *x_right);

/*
I/O:	    Name		      Type	     		  Content
					    					  
[in]	    x1		          int		     	  x coord of ground point of image.
[in]	    y1		          int		     	  y coord of ground point of image.
[in/out]	X	              double*		      X coord on 3D world (m).
[in/out]	Z	              double*		      Z coord on 3D world (m).
[in/out]	detvanish	      double*		      Dist on 3D world (m).

Realized function:
    + Get the X and Z of 3D world and vanish line based on (x1,y1)ground point of image
*/
extern void LDWS_Get_Dist_xz(int x1,int y1,double *X,double *Z,double *detvanish);

extern double LDWS_GetDetaXofWorld(int imagexL, int y);


extern double LDWS_GetXofWorld(int X, int Y);

extern double LDWS_GetXLengthofWorld(int imageXL, int Y);

extern double LDWS_GetXofImage(int X, int Y);

extern double LDWS_GetXLengthofImage(double worldXL, int Y);

extern double LDWS_GetXofWorld_W(int X, int Y);

extern int LDWS_GetFCWSD_th(void);

extern double LDWS_GetVehiclePosX(int x_img, int y_img);

#ifdef __cplusplus
}
#endif

#endif /* LDWS_LIB_H */
