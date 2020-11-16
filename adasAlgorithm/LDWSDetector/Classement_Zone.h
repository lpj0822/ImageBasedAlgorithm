#ifndef _CLASSEMENT_H_
#define _CLASSEMENT_H_

/*
I/O:	    Name		      Type	     		     Content
					    						  
[in]	    M_I_init		  const Modele_Image*	 Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		      Fichier*	             Global parameters to be used in LDWS .
[in/out]	Tab_Indices	      int*		             The zone ID.

[out]	    returned value	  int		             The Num of valid zones.

Realized function:
    + Sort the CX of M_I_init by creasing, save the ID in Tab_Indices
*/
int Classement_Zones(const Modele_Image *M_I_init, Fichier *Donnees, int *Tab_Indices);

/*
I/O:	    Name		      Type	     		     Content
					    						  
[in]	    M_I_init		  const Modele_Image*	 Init Parameters of 2D modele of road (Location in 2D image).
[in/out]	Donnees		      Fichier*	             Global parameters to be used in LDWS .
[in/out]	Tab_Indices	      int*		             The zone ID.
[in]        Star              const int              star factor
[in]        End               const int              end factor

[out]	    returned value	  int		             The Num of valid zones.

Realized function:
    + Sort the CX of M_I_init between [Donnees->NB_INTERVALLES * Star, Donnees->NB_INTERVALLES * End] by creasing, save the ID in Tab_Indices
*/
int Classement_Zones_LR(const Modele_Image *M_I_init, Fichier *Donnees, int *Tab_Indices, const int Star, const int End);

#endif
