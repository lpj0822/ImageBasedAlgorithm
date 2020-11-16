#ifndef _MAJ_H_
#define _MAJ_H_

/*
I/O:	    Name		          Type	     		       Content
					    						  
[in]	    M_I_init		      const Modele_Image*	   Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_new		          Modele_Image*	           Estimated Parameters of 2D modele of road (Location in 2D image).
[in]	    Zone_detect		      const Zone*		       The detected zone.
[in/out]	Donnees		          Fichier*	               Global parameters to be used in LDWS .
[in]	    indice_zone	          int		               The zone index.

Realized function:
    + Update M_I_new->X and M_I_new->CX based on the detected Zone_detect
*/
void M_A_J_Zone(const Modele_Image *M_I_init, Modele_Image *M_I_new, 
				const Zone *Zone_detect, Fichier *Donnees, int indice_zone);

/*
I/O:	    Name		          Type	     		      Content

[in/out]	M_3D_est		      Modele_3D*	          Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in]	    M_3D_init		      const Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in]	    M_I_est		          const Modele_Image*	  Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		          Fichier*	              Global parameters to be used in LDWS .

Realized function:
    + update M_3D_est->X,CX,Param,C_param based on the detected M_I_est
*/
void M_A_J_Param(Modele_3D *M_3D_est,const Modele_3D *M_3D_init, const Modele_Image *M_I_est, Fichier *Donnees);

/*
I/O:	    Name		    Type	     		  Content
					    						 
[in/out]	M_3D_est		Modele_3D*	          Estimated Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in]	    M_3D_init		const Modele_3D*	  Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	M_I_init		Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in]	    M_I_est		    const Modele_Image*   Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		    Fichier*	          Global parameters to be used in LDWS.

Realized function:
    + Tracking based on parameters
	+ if line changed, update M_3D_est->Param[1]; else, update M_3D_est->X and M_3D_est->CX based on tracking, 
	  update the Init param in next frame M_I_init->X and M_I_init->CX
*/
void M_A_J_Suivi3(Modele_3D *M_3D_est, const Modele_3D *M_3D_init,
			      Modele_Image *M_I_init,  Modele_Filter *M_Filter, const Modele_Image *M_I_est, Fichier *Donnees);

#endif
