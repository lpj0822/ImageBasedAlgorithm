#include "data.h"
#include "Matrice.h"
#include <string.h>

void M_identite(int l, double *C);

void SommeAB(double *A, double *B, int l, int c, double *C);

void DifferenceAB(double *A, double *B, int l, int c, double *C);

void ProduitAB(double *A, double *B, int M, int N, int P, double *C);

void TransposeA(double *A, int lA, int cA, double *At);

void InverseA(double *A, int lA);

/*--------------------------------------------------------------------------------*/

/*
 * Midentite 
 */

/*--------------------------------------------------------------------------------*/
void M_identite(int l, double *C)
{
    int i;

    memset(C, 0, l * l * sizeof(double));
    for (i = 0; i < l; ++i)
    {
        C[i * l + i] = 1;
    }
}

/*--------------------------------------------------------------------------------*/

/*
 * SommeAB 24  36
 */

/*--------------------------------------------------------------------------------*/
void SommeAB(double *A, double *B, int l, int c, double *C)
{
	int i;

	for (i = 0; i < l * c; i++)
	{
		C[i] = A[i] + B[i];
	}
}

/*--------------------------------------------------------------------------------*/

/*
 * DifferenceAB 6 24*24
 */

/*--------------------------------------------------------------------------------*/
void DifferenceAB(double *A, double *B, int l, int c, double *C)
{
	int i;

	for (i = 0; i < l * c; i++)
	{
		C[i] = A[i] - B[i];
	}
}

/*--------------------------------------------------------------------------------*/

/*
 * ProduitAB 
 */

/*--------------------------------------------------------------------------------*/
void ProduitAB(double *A, double *B, int M, int N, int P, double *C)
{
    int i, k ,z, j;
	double s;

    for (z = 0; z < M * P; ++z)
    {
        i = z / P;
        j = z % P;
        s = 0;
        for (k = 0; k < N; k++)
        {
            s += A[i * N + k] * B[k * P + j];
        }
        C[i * P + j] = s;
    }
}

/*--------------------------------------------------------------------------------*/

/*
 * TransposeA 
 */

/*--------------------------------------------------------------------------------*/
void TransposeA(double *A, int lA, int cA, double *At)
{
	int l, c;

	for (l = 0; l < lA; l++)
	{
		for (c = 0; c < cA; c++)
		{
			At[c * lA + l] = A[l * cA + c];
		}
	}
}

#define SWAP( a, b )	{ double temp = (a); (a) = (b); (b) = temp; }

/*--------------------------------------------------------------------------------*/

/*
 * Inverse A 
 */

/*--------------------------------------------------------------------------------*/
void InverseA(double *A, int lA)
{
	int indxc[LDWS_NB_PARAM];
	int indxr[LDWS_NB_PARAM];
	int ipiv[LDWS_NB_PARAM];
    int i, icol = 0, irow = 0, j, k, l, ll, tempI;
    double big, dum, pivinv, temp;

    tempI =LDWS_NB_PARAM * sizeof(int);

    memset(ipiv, 0, tempI);

    for (i = 0; i < lA; i++)
	{
		big = 0.0;
        for (j = 0; j < lA; j++)
		{
            if (ipiv[j] != 1)
            {
                for (k = 0; k < lA; k++)
                {
                    if (ipiv[k] == 0)
                    {
                        temp = ABS(A[j + k * lA]);
                        if (temp >= big)
                        {
                            big  = temp;
                            irow = j;
                            icol = k;
                        }
                    }
                }
            }
		}
		++(ipiv[icol]);
		if (irow != icol)
		{
            for (l = 0; l < lA; l++)
			{
                SWAP(A[irow + l * lA], A[icol + l * lA]);
			}
		}

		indxr[i] = irow;
		indxc[i] = icol;
        tempI	 = icol + icol * lA;
        if (A[tempI] == 0.0)
		{
            return;
		}
        pivinv		= 1.0 / A[tempI];
        A[tempI]	= 1.0;
        for (l = icol; l < lA * lA + icol; l += lA)
		{
            A[l] *= pivinv;
		}

        for (ll = 0; ll < lA; ll++)
		{
            if (ll != icol)
            {
                tempI		= ll + icol * lA;
                dum			= A[tempI];
                A[tempI]	= 0.0;
                for (l = 0; l < lA * lA; l += lA)
                {
                    A[ll + l] -= A[icol + l] * dum;
                }
            }

		}
	}

    for (l = lA - 1; l >= 0; l--)
	{
		if (indxr[l] != indxc[l])
		{
            for (k = 0; k < lA; k++)
			{
                SWAP(A[k + indxr[l] * lA], A[k + indxc[l] * lA]);
			}
		}
	}
}

#undef SWAP
#undef ABS
