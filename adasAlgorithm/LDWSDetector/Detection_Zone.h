#ifndef _DETECTIONZONE_H_
#define _DETECTIONZONE_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	     const unsigned char*	  The input image buffer.

[out]	    returned	          int	              if 0 failed;else return the detected points num

Realized function:
    + line fitting based on the detected points, and update Zone_detect->X and Zone_detect->CX
*/
int Detection_Zone(Zone *Zone_detect, Fichier *Donnees, const unsigned char *Tab_Image);

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .
[in]	    Tab_Image	     const unsigned char*	  The input image buffer.

[out]	    returned	          int	              if 0 failed;else return the detected points num

Realized function:
    + line fitting based on the detected points
*/
int Detection_Zone_LR(Zone *Zone_detect, Fichier *Donnees, const unsigned char *Tab_Image);

#endif
