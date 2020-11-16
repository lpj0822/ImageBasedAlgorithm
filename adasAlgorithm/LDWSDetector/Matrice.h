#ifndef _MATRICE_H_
#define _MATRICE_H_

extern void M_identite(int l, double *C);

extern void SommeAB(double *A, double *B, int l, int c, double *C);

extern void DifferenceAB(double *A, double *B, int l, int c, double *C);

extern void ProduitAB(double *A, double *B, int lA, int cA, int cB, double *C);

extern void TransposeA(double *A, int lA, int cA, double *At);

extern void InverseA(double *A, int lA);

extern void AfficheMatrice(double *M, int nbl, int nbc, char *titre);

#endif
