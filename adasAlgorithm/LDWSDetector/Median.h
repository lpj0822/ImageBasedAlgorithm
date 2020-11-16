#ifndef _MEDIAN_H_
#define _MEDIAN_H_

/*
I/O:	    Name		          Type	     		  Content
					    						  
[in/out]	tab_x		          int*		          The detected x coord of points.
[in/out]	tab_y		          int*		          The detected y coord of points.
[in/out]	nb_pts	              int*	              The detected points num
[in/out]	Zone_detect		      Zone*		          The detected zone.
[in/out]	Donnees		          Fichier*	          Global parameters to be used in LDWS .

[out]	    returned	          int	              if 0 failed

Realized function:
    + line fitting based on the detected points, and update Zone_detect->X and Zone_detect->CX
*/
int Median(int *tab_x, int *tab_y, int *nb_pts, Zone *Zone_detect, Fichier *Donnees);

#endif
