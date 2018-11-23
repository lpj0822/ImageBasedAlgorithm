#include "assignment_problem_solver.h"
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "helpers/utility.h"
#include "helpers/utility_data_structure.h"

static void buildAssignmentVector(const Bool *starMatrix, const int nOfRows, const int nOfColumns, int *assignment);
static void computeAssignmentCost(const double *distMatrix, const int nOfRows, const int *assignment, double *cost);
static void step2a(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                   Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
static void step2b(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                   Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
static void step3(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                  Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
static void step4(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
           Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
static void step5(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                  Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim);


//Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
static void computeOptimalAssignment(const double *distMatrixIn, const int nOfRows, const int nOfColumns, int *assignment, double *cost);

//Computes a suboptimal solution. Good for cases with many forbidden assignments.
static void computeSuboptimalAssignment1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
//Greedy Algorithm
static void computeSuboptimalAssignment2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);

/*assign M tasks to N performers*/
double assignmentProblemSolver(const double* costMatrix, const int N, const int M, int* assignmentResult)
{
    double cost = 0;
    computeOptimalAssignment(costMatrix, N, M, assignmentResult, &cost);
    return cost;
}


//Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
static void computeOptimalAssignment(const double *distMatrixIn, const int nOfRows, const int nOfColumns, int *assignment, double *cost)
{
    double *distMatrix;
    double *distMatrixTemp;
    double *distMatrixEnd;
    double *columnEnd;
    double  value;
    double  minValue;

    Bool *coveredColumns;
    Bool *coveredRows;
    Bool *starMatrix;
    Bool *newStarMatrix;
    Bool *primeMatrix;

    int nOfElements;
    int minDim;
    int row;
    int col;

    // Init
    *cost = 0;
    for (row = 0; row<nOfRows; row++)
    {
        assignment[row] = -1;
    }

    // Generate distance matrix
    // and check matrix elements positiveness :)

    // Total elements number
    nOfElements = nOfRows * nOfColumns;
    // Memory allocation
    distMatrix = (double *)malloc(nOfElements * sizeof(double));
    // Pointer to last element
    distMatrixEnd = distMatrix + nOfElements;

    //
    for (row = 0; row<nOfElements; row++)
    {
        value = distMatrixIn[row];
        if (value < 0)
        {
            printf("All matrix elements have to be non-negative.\n");
        }
        distMatrix[row] = value;
    }

    // Memory allocation
    coveredColumns = (Bool *)calloc(nOfColumns, sizeof(Bool));
    coveredRows = (Bool *)calloc(nOfRows, sizeof(Bool));
    starMatrix = (Bool *)calloc(nOfElements, sizeof(Bool));
    primeMatrix = (Bool *)calloc(nOfElements, sizeof(Bool));
    newStarMatrix = (Bool *)calloc(nOfElements, sizeof(Bool)); /* used in step4 */

    /* preliminary steps */
    if (nOfRows <= nOfColumns)
    {
        minDim = nOfRows;
        for (row = 0; row<nOfRows; row++)
        {
            /* find the smallest element in the row */
            distMatrixTemp = distMatrix + row;
            minValue = *distMatrixTemp;
            distMatrixTemp += nOfRows;
            while (distMatrixTemp < distMatrixEnd)
            {
                value = *distMatrixTemp;
                if (value < minValue)
                {
                    minValue = value;
                }
                distMatrixTemp += nOfRows;
            }
            /* subtract the smallest element from each element of the row */
            distMatrixTemp = distMatrix + row;
            while (distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }
        /* Steps 1 and 2a */
        for (row = 0; row<nOfRows; row++)
        {
            for (col = 0; col<nOfColumns; col++)
            {
                if (distMatrix[row + nOfRows*col] == 0)
                {
                    if (!coveredColumns[col])
                    {
                        starMatrix[row + nOfRows*col] = TRUE;
                        coveredColumns[col] = TRUE;
                        break;
                    }
                }
            }
        }
    }
    else /* if(nOfRows > nOfColumns) */
    {
        minDim = nOfColumns;
        for (col = 0; col<nOfColumns; col++)
        {
            /* find the smallest element in the column */
            distMatrixTemp = distMatrix + nOfRows*col;
            columnEnd = distMatrixTemp + nOfRows;
            minValue = *distMatrixTemp++;
            while (distMatrixTemp < columnEnd)
            {
                value = *distMatrixTemp++;
                if (value < minValue)
                {
                    minValue = value;
                }
            }
            /* subtract the smallest element from each element of the column */
            distMatrixTemp = distMatrix + nOfRows*col;
            while (distMatrixTemp < columnEnd)
            {
                *distMatrixTemp++ -= minValue;
            }
        }
        /* Steps 1 and 2a */
        for (col = 0; col<nOfColumns; col++)
        {
            for (row = 0; row<nOfRows; row++)
            {
                if (distMatrix[row + nOfRows*col] == 0)
                {
                    if (!coveredRows[row])
                    {
                        starMatrix[row + nOfRows*col] = TRUE;
                        coveredColumns[col] = TRUE;
                        coveredRows[row] = TRUE;
                        break;
                    }
                }
            }
        }

        for (row = 0; row<nOfRows; row++)
        {
            coveredRows[row] = FALSE;
        }
    }
    /* move to step 2b */
    step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    /* compute cost and remove invalid assignments */
    computeAssignmentCost(distMatrixIn, nOfRows, assignment, cost);
    /* free allocated memory */
    free(distMatrix);
    free(coveredColumns);
    free(coveredRows);
    free(starMatrix);
    free(primeMatrix);
    free(newStarMatrix);
    return;
}

static void buildAssignmentVector(const Bool *starMatrix, const int nOfRows, const int nOfColumns, int *assignment)
{
    int row = 0;
    int col = 0;
    for (row = 0; row < nOfRows; row++)
    {
        for (col = 0; col<nOfColumns; col++)
        {
            if (starMatrix[row + nOfRows*col])
            {
                assignment[row] = col;
                break;
            }
        }
    }
}

static void computeAssignmentCost(const double *distMatrix, const int nOfRows, const int *assignment, double *cost)
{
    int row, col;
    for (row = 0; row<nOfRows; row++)
    {
        col = assignment[row];
        if (col >= 0)
        {
            *cost += distMatrix[row + nOfRows*col];
        }
    }
}

static void step2a(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                   Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    Bool *starMatrixTemp, *columnEnd;
    int col;
    /* cover every column containing a starred zero */
    for (col = 0; col<nOfColumns; col++)
    {
        starMatrixTemp = starMatrix + nOfRows*col;
        columnEnd = starMatrixTemp + nOfRows;
        while (starMatrixTemp < columnEnd)
        {
            if (*starMatrixTemp++)
            {
                coveredColumns[col] = TRUE;
                break;
            }
        }
    }
    /* move to step 3 */
    step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


static void step2b(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                   Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    int col, nOfCoveredColumns;
    /* count covered columns */
    nOfCoveredColumns = 0;
    for (col = 0; col<nOfColumns; col++)
    {
        if (coveredColumns[col])
        {
            nOfCoveredColumns++;
        }
    }
    if (nOfCoveredColumns == minDim)
    {
        /* algorithm finished */
        buildAssignmentVector(starMatrix, nOfRows, nOfColumns, assignment);
    }
    else
    {
        /* move to step 3 */
        step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }
}


static void step3(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                  Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    Bool zerosFound;
    int row, col, starCol;
    zerosFound = TRUE;
    while (zerosFound)
    {
        zerosFound = FALSE;
        for (col = 0; col<nOfColumns; col++)
        {
            if (!coveredColumns[col])
            {
                for (row = 0; row<nOfRows; row++)
                {
                    if ((!coveredRows[row]) && (distMatrix[row + nOfRows*col] == 0))
                    {
                        /* prime zero */
                        primeMatrix[row + nOfRows*col] = TRUE;
                        /* find starred zero in current row */
                        for (starCol = 0; starCol<nOfColumns; starCol++)
                        if (starMatrix[row + nOfRows*starCol])
                        {
                            break;
                        }
                        if (starCol == nOfColumns) /* no starred zero found */
                        {
                            /* move to step 4 */
                            step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                            return;
                        }
                        else
                        {
                            coveredRows[row] = TRUE;
                            coveredColumns[starCol] = FALSE;
                            zerosFound = TRUE;
                            break;
                        }
                    }
                }
            }
        }
    }
    /* move to step 5 */
    step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}


static void step4(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                  Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
    int n, starRow, starCol, primeRow, primeCol;
    int nOfElements = nOfRows*nOfColumns;
    /* generate temporary copy of starMatrix */
    for (n = 0; n<nOfElements; n++)
    {
        newStarMatrix[n] = starMatrix[n];
    }
    /* star current zero */
    newStarMatrix[row + nOfRows*col] = TRUE;
    /* find starred zero in current column */
    starCol = col;
    for (starRow = 0; starRow<nOfRows; starRow++)
    {
        if (starMatrix[starRow + nOfRows*starCol])
        {
            break;
        }
    }
    while (starRow<nOfRows)
    {
        /* unstar the starred zero */
        newStarMatrix[starRow + nOfRows*starCol] = FALSE;
        /* find primed zero in current row */
        primeRow = starRow;
        for (primeCol = 0; primeCol<nOfColumns; primeCol++)
        {
            if (primeMatrix[primeRow + nOfRows*primeCol])
            {
                break;
            }
        }
        /* star the primed zero */
        newStarMatrix[primeRow + nOfRows*primeCol] = TRUE;
        /* find starred zero in current column */
        starCol = primeCol;
        for (starRow = 0; starRow<nOfRows; starRow++)
        {
            if (starMatrix[starRow + nOfRows*starCol])
            {
                break;
            }
        }
    }
    /* use temporary copy as new starMatrix */
    /* delete all primes, uncover all rows */
    for (n = 0; n<nOfElements; n++)
    {
        primeMatrix[n] = FALSE;
        starMatrix[n] = newStarMatrix[n];
    }
    for (n = 0; n<nOfRows; n++)
    {
        coveredRows[n] = FALSE;
    }
    /* move to step 2a */
    step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

static void step5(int *assignment, double *distMatrix, Bool *starMatrix, Bool *newStarMatrix,
                  Bool *primeMatrix, Bool *coveredColumns, Bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
    double h, value;
    int row, col;
    /* find smallest uncovered element h */
    h = DBL_MAX;
    for (row = 0; row<nOfRows; row++)
    {
        if (!coveredRows[row])
        {
            for (col = 0; col<nOfColumns; col++)
            {
                if (!coveredColumns[col])
                {
                    value = distMatrix[row + nOfRows*col];
                    if (value < h)
                    {
                        h = value;
                    }
                }
            }
        }
    }
    /* add h to each covered row */
    for (row = 0; row<nOfRows; row++)
    {
        if (coveredRows[row])
        {
            for (col = 0; col<nOfColumns; col++)
            {
                distMatrix[row + nOfRows*col] += h;
            }
        }
    }
    /* subtract h from each uncovered column */
    for (col = 0; col<nOfColumns; col++)
    {
        if (!coveredColumns[col])
        {
            for (row = 0; row<nOfRows; row++)
            {
                distMatrix[row + nOfRows*col] -= h;
            }
        }
    }
    /* move to step 3 */
    step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

//Computes a suboptimal solution. Good for cases with many forbidden assignments.
static void computeSuboptimalAssignment1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    Bool infiniteValueFound, finiteValueFound;
    Bool repeatSteps, allSinglyValidated, singleValidationFound;
    int n, row, col, tmpRow, tmpCol, nOfElements;
    int *nOfValidObservations, *nOfValidTracks;
    double value, minValue, *distMatrix;


    /* make working copy of distance Matrix */
    nOfElements = nOfRows * nOfColumns;
    distMatrix = (double *)malloc(nOfElements * sizeof(double));
    for (n = 0; n<nOfElements; n++)
    {
        distMatrix[n] = distMatrixIn[n];
    }
    /* initialization */
    *cost = 0;

    for (row = 0; row<nOfRows; row++)
    {
        assignment[row] = -1;
    }

    /* allocate memory */
    nOfValidObservations = (int *)calloc(nOfRows, sizeof(int));
    nOfValidTracks = (int *)calloc(nOfColumns, sizeof(int));

    /* compute number of validations */
    infiniteValueFound = FALSE;
    finiteValueFound = FALSE;
    for (row = 0; row<nOfRows; row++)
    {
        for (col = 0; col<nOfColumns; col++)
        {
            if (distMatrix[row + nOfRows*col] != DBL_MAX)
            {
                nOfValidTracks[col] += 1;
                nOfValidObservations[row] += 1;
                finiteValueFound = TRUE;
            }
            else
                infiniteValueFound = TRUE;
        }
    }

    if (infiniteValueFound)
    {
        if (!finiteValueFound)
        {
            return;
        }
        repeatSteps = TRUE;

        while (repeatSteps)
        {
            repeatSteps = FALSE;

            /* step 1: reject assignments of multiply validated tracks to singly validated observations		 */
            for (col = 0; col<nOfColumns; col++)
            {
                singleValidationFound = FALSE;
                for (row = 0; row<nOfRows; row++)
                    if ((distMatrix[row + nOfRows*col] != DBL_MAX) && (nOfValidObservations[row] == 1))
                    {
                        singleValidationFound = TRUE;
                        break;
                    }

                if (singleValidationFound)
                {
                    for (row = 0; row<nOfRows; row++)
                        if ((nOfValidObservations[row] > 1) && distMatrix[row + nOfRows*col] != DBL_MAX)
                        {
                            distMatrix[row + nOfRows*col] = DBL_MAX;
                            nOfValidObservations[row] -= 1;
                            nOfValidTracks[col] -= 1;
                            repeatSteps = FALSE;
                        }
                }
            }

            /* step 2: reject assignments of multiply validated observations to singly validated tracks */
            if (nOfColumns > 1)
            {
                for (row = 0; row<nOfRows; row++)
                {
                    singleValidationFound = FALSE;
                    for (col = 0; col<nOfColumns; col++)
                    {
                        if (distMatrix[row + nOfRows*col] != DBL_MAX && (nOfValidTracks[col] == 1))
                        {
                            singleValidationFound = TRUE;
                            break;
                        }
                    }

                    if (singleValidationFound)
                    {
                        for (col = 0; col<nOfColumns; col++)
                        {
                            if ((nOfValidTracks[col] > 1) && distMatrix[row + nOfRows*col] != DBL_MAX)
                            {
                                distMatrix[row + nOfRows*col] = DBL_MAX;
                                nOfValidObservations[row] -= 1;
                                nOfValidTracks[col] -= 1;
                                repeatSteps = TRUE;
                            }
                        }
                    }
                }
            }
        } /* while(repeatSteps) */

        /* for each multiply validated track that validates only with singly validated  */
        /* observations, choose the observation with minimum distance */
        for (row = 0; row<nOfRows; row++)
        {
            if (nOfValidObservations[row] > 1)
            {
                allSinglyValidated = TRUE;
                minValue = DBL_MAX;
                for (col = 0; col<nOfColumns; col++)
                {
                    value = distMatrix[row + nOfRows*col];
                    if (value != DBL_MAX)
                    {
                        if (nOfValidTracks[col] > 1)
                        {
                            allSinglyValidated = FALSE;
                            break;
                        }
                        else if ((nOfValidTracks[col] == 1) && (value < minValue))
                        {
                            tmpCol = col;
                            minValue = value;
                        }
                    }
                }

                if (allSinglyValidated)
                {
                    assignment[row] = tmpCol;
                    *cost += minValue;
                    for (n = 0; n<nOfRows; n++)
                    {
                        distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
                    }
                    for (n = 0; n<nOfColumns; n++)
                    {
                        distMatrix[row + nOfRows*n] = DBL_MAX;
                    }
                }
            }
        }

        /* for each multiply validated observation that validates only with singly validated  */
        /* track, choose the track with minimum distance */
        for (col = 0; col<nOfColumns; col++)
        {
            if (nOfValidTracks[col] > 1)
            {
                allSinglyValidated = TRUE;
                minValue = DBL_MAX;
                for (row = 0; row<nOfRows; row++)
                {
                    value = distMatrix[row + nOfRows*col];
                    if (value != DBL_MAX)
                    {
                        if (nOfValidObservations[row] > 1)
                        {
                            allSinglyValidated = FALSE;
                            break;
                        }
                        else if ((nOfValidObservations[row] == 1) && (value < minValue))
                        {
                            tmpRow = row;
                            minValue = value;
                        }
                    }
                }

                if (allSinglyValidated)
                {
                    assignment[tmpRow] = col;
                    *cost += minValue;
                    for (n = 0; n<nOfRows; n++)
                        distMatrix[n + nOfRows*col] = DBL_MAX;
                    for (n = 0; n<nOfColumns; n++)
                        distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
                }
            }
        }
    } /* if(infiniteValueFound) */


    /* now, recursively search for the minimum element and do the assignment */
    while (1)
    {
        /* find minimum distance observation-to-track pair */
        minValue = DBL_MAX;
        for (row = 0; row<nOfRows; row++)
        for (col = 0; col<nOfColumns; col++)
        {
            value = distMatrix[row + nOfRows*col];
            if (value != DBL_MAX && (value < minValue))
            {
                minValue = value;
                tmpRow = row;
                tmpCol = col;
            }
        }

        if (minValue != DBL_MAX)
        {
            assignment[tmpRow] = tmpCol;
            *cost += minValue;
            for (n = 0; n<nOfRows; n++)
            {
                distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
            }
            for (n = 0; n<nOfColumns; n++)
            {
                distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
            }
        }
        else
            break;

    }

    /* free allocated memory */
    free(nOfValidObservations);
    nOfValidObservations = NULL;
    free(nOfValidTracks);
    nOfValidTracks = NULL;
}

//Greedy Algorithm
static void computeSuboptimalAssignment2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
    int n, row, col, tmpRow, tmpCol, nOfElements;
    double value, minValue, *distMatrix;


    /* make working copy of distance Matrix */
    nOfElements = nOfRows * nOfColumns;
    distMatrix = (double *)malloc(nOfElements * sizeof(double));
    for (n = 0; n<nOfElements; n++)
    {
        distMatrix[n] = distMatrixIn[n];
    }

    /* initialization */
    *cost = 0;
    for (row = 0; row<nOfRows; row++)
    {
        assignment[row] = -1;
    }

    /* recursively search for the minimum element and do the assignment */
    while (1)
    {
        /* find minimum distance observation-to-track pair */
        minValue = DBL_MAX;
        for (row = 0; row<nOfRows; row++)
        for (col = 0; col<nOfColumns; col++)
        {
            value = distMatrix[row + nOfRows*col];
            if (value != DBL_MAX && (value < minValue))
            {
                minValue = value;
                tmpRow = row;
                tmpCol = col;
            }
        }

        if (minValue != DBL_MAX)
        {
            assignment[tmpRow] = tmpCol;
            *cost += minValue;
            for (n = 0; n<nOfRows; n++)
            {
                distMatrix[n + nOfRows*tmpCol] = DBL_MAX;
            }
            for (n = 0; n<nOfColumns; n++)
            {
                distMatrix[tmpRow + nOfRows*n] = DBL_MAX;
            }
        }
        else
            break;

    }

    free(distMatrix);
}
