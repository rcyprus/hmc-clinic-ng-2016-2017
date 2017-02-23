// inputs.h
// Contains helper functions to create necessary inputs to quadrotor SSE

#include "inputs.h"

/* 
 * Creates a T x PT matrix for with the indexed variable p
 */
void createI(int T, int P, double* I);

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 */
void createCA(int T, int P, int n, double* A, double* C, double* CA);

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 */
void createYBu(double* A, double* B, double* C,
               int T, int P, int n, int m,
               double* Y, double* U, double* YBu);

/* 
 * Raises the square matrix A with a size len x len to the power of T
 */
void power(double* A, int len, int T, double* AT);

/* 
 * Multiplies together matrices A and B (sizes must be compatible)
 */
void multiply(double* A, int rowsA, int colsA,
              double* B, int rowsB, int colsB,
              double* AB);

/* 
 * Takes the dot product of vectors x and y (vectors must have length len)
 */
double dot(double* x, double* y, int len);

/* 
 * Adds vectors x and y together (both vectors must of of length len)
 */
void add(double* x, double* y, int len, double* xplusy);


