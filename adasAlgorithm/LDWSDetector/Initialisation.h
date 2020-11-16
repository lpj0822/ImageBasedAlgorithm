#ifndef _INITIALISATION_H_
#define _INITIALISATION_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_init		      Modele_Image*	      Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_3D_init		      Modele_3D*	      Init Parameters of 3D modele of road (Location in 2D image and position in 3D world).
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + Init the values for parameters
*/
void Initialisation(Modele_Image *M_I_init, Modele_Image *M_I_est, Modele_3D *M_3D_init, const Fichier *Donnees);

#endif
