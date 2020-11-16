#ifndef _CARAC_H_
#define _CARAC_H_

/*
I/O:	    Name		    Type	         Content
					    						  
[in]	    acces		    const char*	     input file name.
[in]	    chaine		    const char*		 chars wants to found.
[in]	    type		    const char*		 The type of paramter to given.
[in/out]	ptr		        double*		     The found double value.

[out]	    return value    int		         If double paramter is found in File return 1; else return 0.

Realized function:
    + Find the double paramter in the file
*/
int LectureParametresdouble(const char *acces, const char *chaine, const char *type, double *ptr);

/*
I/O:	    Name		    Type	         Content
					    						  
[in]	    acces		    const char*	     input file name.
[in]	    chaine		    const char*		 chars wants to found.
[in]	    type		    const char*		 The type of paramter to given.
[in/out]	ptr		        double*		     The found int value.

[out]	    return value    int		         If int paramter is found in File return 1; else return 0.

Realized function:
    + Find the int paramter in the file
*/
int LectureParametresInt(const char *acces, const char *chaine, const char *type, int *ptr);

#endif
