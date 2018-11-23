/*
 *Hungarian algorithm with task allocation procedures
 */
#ifndef ASSIGNMENT_PROBLEM_SOLVER_H
#define ASSIGNMENT_PROBLEM_SOLVER_H

/*assign M tasks to N performers*/
double assignmentProblemSolver(const double* costMatrix, const int N, const int M, int* assignmentResult);

#endif // ASSIGNMENT_PROBLEM_SOLVER_H
