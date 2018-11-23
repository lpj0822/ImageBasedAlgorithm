#include "test_tracking.h"

/*
//test
void main(void)
{
// Matrix size
int N=8; // tracks
int M=9; // detects
// Random numbers generator initialization
std::ssrand (time(NULL));
// Distance matrix N-th track to M-th detect.
std::vector< vector<double> > Cost(N,vector<double>(M));
// Fill matrix with random values
for(int i=0; i<N; i++)
{
for(int j=0; j<M; j++)
{
Cost[i][j] = (double)(rand()%1000)/1000.0;
std::cout << Cost[i][j] << "\t";
}
std::cout << std::endl;
}

AssignmentProblemSolver APS;

std::vector<int> Assignment;

std::cout << APS.Solve(Cost,Assignment) << std::endl;

// Output the result
for(int x=0; x<N; x++)
{
std::cout << x << ":" << Assignment[x] << "\t";
}

getchar();
}
*/
