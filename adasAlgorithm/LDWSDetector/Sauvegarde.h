#ifndef _SAUVEGARDE_H_
#define _SAUVEGARDE_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	M_I_new		          Modele_Image*	      Calculated Parameters of 2D modele of road (Location in 2D image).
[in/out]	M_I_est		          Modele_Image*	      Estimated Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

Realized function:
    + decide if M_I_new satisfied the Proba, if satisfied update M_I_est.
*/
void Sauvegarde(Modele_Image *M_I_new, Modele_Image *M_I_est, Fichier *Donnees);

#endif
