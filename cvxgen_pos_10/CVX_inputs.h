// inputs.h
// Contains helper functions to create necessary inputs to quadrotor SSE

#ifdef __cplusplus
    extern"C" {
#endif

#include "matrix-helper.h"

#ifndef INPUTS_H_INCLUDED
#define INPUTS_H_INCLUDED 1

// Define constants
#define N 4 // N = states
#define P 14 // P = sensors
#define T 10 // T = T
#define M 5 // M = inputs

// Define sparsity
#define nonZeroEntries 16

// Constant Matrices
double A[N*N]; // = {1,0,0,0,0,0.95123,0,0,0,0,0.95123,0,0.097541,0,0,0.95123}; // nxn
double B[N*M]; // = {2.9506e-08,0,0,5.8525e-07,2.9506e-08,0,0,5.8525e-07,2.9506e-08,0,0,5.8525e-07,2.9506e-08,0,0,5.8525e-07,-0.0049177,0,0,-0.097541};  // nxm
double C[P*N]; // = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,0,0,1}; // Pxn

// Update contents after each timeStep
double CA[P*T*N]; // PTxn
double YBu[P*T];  // PTx1
double U[M*T];    // MxT
double Y[P*T];
double y[P];
double x[N]; // state vector

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
void updateYBu(int timeStep, double* yin, double* Uin);

/* 
 * Given the initial state output from CVX and U (vector of global inputs),
 * propagate system dynamics to obtain the previous system state
 */
void propagateDynamics(int timeStep, double* U, double* x);

/* 
 * Raises the square matrix A with a size len x len to the power of T
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT);

#endif


#ifdef __cplusplus
    }
#endif


