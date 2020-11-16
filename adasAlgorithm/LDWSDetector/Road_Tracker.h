#ifndef _ROAD_TRACKER_H_
#define _ROAD_TRACKER_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in]	    mode		          int		     	  Index the mode of road.
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_3D_est		      Modele_3D*	      Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in/out]	L_output		      LDWS_Output*	      Output parameters model.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    fichier_init	      const char*		  Config files to Init paramters.
[in]	    fichier_init_custom	  const char*		  Customer config files to Init paramters.

Realized function:
    + Init the values of parameters
*/
void LDWS_InitRoadTracker(int mode, 
						  Modele_Image *M_I_init, Modele_Image *M_I_est,
						  Modele_3D *M_3D_init, Modele_3D *M_3D_est,
						  Modele_Filter *M_Filter, LDWS_Output *L_output, 
						  Fichier *Donnees, const char *fichier_init,const char *fichier_init_custom);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_3D_est		      Modele_3D*	      Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in/out]	L_output		      LDWS_Output*	      Output parameters model.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	       const unsigned char*	  Input image buffer.

Realized function:
    + The main realization of LDWS
*/
void LDWS_RoadTracker(Modele_Image *M_I_init, Modele_Image *M_I_est, 
					  Modele_3D *M_3D_init, Modele_3D *M_3D_est,
					  Modele_Filter *M_Filter, LDWS_Output *L_output,
					  Fichier *Donnees, const unsigned char *Tab_Image);

#endif
