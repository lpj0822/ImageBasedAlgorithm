#ifndef _INIT_STRUCT_h
#define _INIT_STRUCT_h

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    fichier	              const char*		  Config files to Init paramters.
[in]	    fichierCustom	      const char*		  Customer config files to Init paramters.

[out]       returned value        int                 if Init succeeded return 1; else return 0; 

Realized function:
    + Read the config files and Init Donnees
*/
int Get_Fichier(Fichier *Donnees,const char *fichier, const char *fichierCustom);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Init Donnees
*/
void Init_Modele_Fichier(Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_3D_est		      Modele_3D*	      Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Malloc the memory of M_3D_init, M_3D_est and M_Filter
*/
void Allocation_Modele_Struct(Modele_3D *M_3D_init, Modele_3D *M_3D_est, 
							  Modele_Filter *M_Filter, const Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_3D_est		      Modele_3D*	      Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + In the initialization, given the value of M_3D_init->Param and M_3D_init->C_Param;
	  If lines not found, Init the M_I_init->X and CX based on M_3D_init
*/
void Init_Modele_3D(Modele_Image *M_I_init, Modele_3D *M_3D_init, 
					LDWS_Output *L_output, Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	L_output		      LDWS_Output*	      Output parameters model.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.
[in]	    M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).

Realized function:
    + Init the values for L_output
*/
void Init_Modele_Output(LDWS_Output *L_output, const Fichier *Donnees, const Modele_3D *M_3D_init);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.
[in]	    Donnees		          Fichier*	          Global parameters to be used in LDWS.

Realized function:
    + Init the values for M_Filter
*/
void Init_Modele_Filter(Modele_Filter *M_Filter, const Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Init the related parameters in M_I_init and M_I_est in each frame
*/
void Init_Modele_Image(Modele_Image *M_I_init, Modele_Image *M_I_est, Fichier *Donnees);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	L_output		      LDWS_Output*	      Output parameters model.

Realized function:
    + Free the values for L_output
*/
void Free_Modele_OutPut(LDWS_Output *L_output);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_3D_est		      Modele_3D*	      Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_Filter		      Modele_Filter*	  Median filter parameters model.

Realized function:
    + Free the memory of M_3D_init, M_3D_est and M_Filter
*/
void Free_Modele_Struct(Modele_3D *M_3D_init, Modele_3D *M_3D_est, Modele_Filter *M_Filter);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS.

Realized function:
    + Free the memory of Donnees
*/
void Free_Modele_Fichier(Fichier *Donnees);


extern void Modele_TRACTEUR(Modele_3D *M_3D_init, Fichier *Donnees);

#endif
