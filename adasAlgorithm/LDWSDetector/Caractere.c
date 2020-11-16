#include "data.h"
#include "LDWS_Interface.h"
#include <stdio.h>
#include <string.h>

/*
I/O:	    Name		    Type	   Content
					    						  
[in/out]	fp		        FILE*	   Files.
[in]	    str		        char*	   Chars wants to found.

[out]	    return value    int		   If char found in File return 1; else return 0.

Realized function:
    + Find the chars in the file
*/
static int RechercheChaine(FILE * fp, const char *str);

/******************************************************************************************/

/*
Function process:
	+ Find the chars in the file
	Fan-in : 
	        + LectureParametresdouble()
			+ LectureParametresInt()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
static int RechercheChaine(FILE * fp, const char *str)
{
	int s = 0;
	short i = 0, longueur_str = 0;
	char trouve, temp[256];

	if (fp != NULL)
	{
		i				= 0;
		trouve			= 0;
		longueur_str	= strlen(str);
		if (longueur_str != 0)
		{
			do
			{
				s			= getc(fp);
				temp[i++]	= s;
			}
			while ((i < longueur_str) && (s != EOF));

			if (i == longueur_str)
			{
				temp[longueur_str] = '\0';
				if (s != EOF)
				{
					do
					{
						if (strcmp(str, temp) == 0)
						{
							trouve = 1;
						}
						else
						{
							for (i = 1; i < longueur_str; i++)
							{
								temp[i - 1] = temp[i];
							}
							s						= getc(fp);
							temp[longueur_str - 1]	= s;
							temp[longueur_str]		= '\0';
						}
					}
					while ((!trouve) && (s != EOF));
				}
				return (trouve);
			}
		}
		else
		{
			return (0);
		}
	}
	else
	{
		return (0);
	}

	return (0);
}

/*
Function process:
	+ Find the double paramter in the file
	Fan-in : 
	        + Get_Fichier()
			+ RechercheChaine()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int LectureParametresdouble(const char *acces, const char *chaine, const char *type, double *ptr)
{
	FILE *fp = NULL;
	char retour;

	if ((fp = fopen(acces, "r")) != NULL)
	{
		if (RechercheChaine(fp, chaine))
		{
            if (fscanf(fp, type, ptr) == 1) /* si on a effectivement lu une donnee */
            {
				retour = 1;
			}
			else
			{
				retour = 0;
			}
		}
		else
		{
			retour = 0;
		}
		fclose(fp);
	}
	else
	{
		retour = 0;
	}
	return (retour);
}

/*
Function process:
	+ Find the int paramter in the file
	Fan-in : 
	        + Get_Fichier()
			+ RechercheChaine()
	Fan-out:
	        + N/A
	ATTENTION: __________
*/
int LectureParametresInt(const char *acces, const char *chaine, const char *type, int *ptr)
{
	FILE *fp = NULL;
	char retour;

	if ((fp = fopen(acces, "r")) != NULL)
	{
		if (RechercheChaine(fp, chaine))
		{
			/* si on a effectivement lu une donnee */
			if (fscanf(fp, type, ptr) == 1)
			{
				retour = 1;
			}
			else
			{
				retour = 0;
			}
		}
		else
		{
			retour = 0;
		}
		fclose(fp);
	}
	else
	{
		retour = 0;
	}
	
	return (retour);
}
