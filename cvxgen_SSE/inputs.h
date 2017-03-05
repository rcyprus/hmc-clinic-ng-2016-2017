// inputs.h
// Contains helper functions to create necessary inputs to quadrotor SSE

#ifndef INPUTS_H_INCLUDED
#define INPUTS_H_INCLUDED 1

// Define constants
#define n 5 // states
#define P 5 // sensors
#define T 3  // timesteps
#define m 3  // inputs (4 motors + gravity)

// Constant Matrices
double A[n*n]; // nxn
double B[n*m]; // nxm
double C[P*n]; // Pxn

// Constant (but need to populate)
int I[P+1][T*T*P]; // P+1 b/c CVXGEN may use only the latter 5 rows ...

// Update contents after each timeStep
double CA[P*T*n]; // PTxn
double YBu[P*T]; // PTx1
double U[m*T];
double y[P];

// Store system state
double x[n];

/* 
 * Creates a T x PT matrix for with the indexed variable p
 */
void setupI(void);

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 * Updates the specified "section" of CA matrix (global)
 */
void updateCA(int timeStep);

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 * Inputs: timeStep: only updating at a given timeStep
 *         y: sensor outputs at current timeStep (Px1)
 *         U: control inputs (all time) (mTx1)
 * Output: YBu (global matrix) (PTx1)
 */
void updateYBu(int timeStep, double* y, double* U);

void propagateDynamics(int timeStep, double* U, double* x);

/* 
 * Raises the square matrix A with a size len x len to the power of T
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT);

/* 
 * Multiplies together matrices A and B (sizes must be compatible)
 */
void multiply(double* X, int rowsX, int colsX,
              double* Y, int rowsY, int colsY,
              double* XY);

/* 
 * Takes the dot product of vectors x and y (vectors must have length len)
 */
double dot(double* x, double* y, int len);

/* 
 * Adds vectors x and y together (both vectors must of of length len)
 */
void add(double* x, double* y, int len, double* xplusy);

void sub(double* x, double* y, int len, double* xminy); 

/*
 * Debugging print function (only 2D arrays)
 */
void printArrayDouble(double* array, int rows, int cols);

/*
 * Debugging print function (only 2D int arrays)
 */
void printArrayInt(int* array, int rows, int cols);

/*
 * Read in 1D array from file (w/ comma delimiter) 
 */
void readArrayFromFile(const char* file_name, double* array);

/*
 * Simple read function to test fscan
 */
void simpleFile(void);



#endif

