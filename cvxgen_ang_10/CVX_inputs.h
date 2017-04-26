// inputs.h
// Contains helper functions to create necessary inputs to quadrotor SSE

#ifdef __cplusplus
    extern"C" {
#endif

#include "matrix-helper.h"

#ifndef INPUTS_H_INCLUDED
#define INPUTS_H_INCLUDED 1

// Define constants
#define numStates 6 // N = states
#define numSensors 24 // P = sensors
#define timeSteps 10 // T = timesteps
#define numInputs 5 // M = inputs

// Define sparsity
#define nonZeroEntries 36

// Constant Matrices
double A[numStates*numStates] = {1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0.1,0,0,1,0,0,0,0.1,0,0,1,0,0,0,0.1,0,0,1}; // nxn
double B[numStates*numInputs] = {7.5e-07,0,5e-08,1.5e-05,0,1e-06,0,7.5e-07,-5e-08,0,1.5e-05,-1e-06,-7.5e-07,0,5e-08,-1.5e-05,0,1e-06,0,-7.5e-07,-5e-08,0,-1.5e-05,-1e-06,0,0,0,0,0,0}; // nxm
double C[numSensors*numStates] = {1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1}; // Pxn

// Update contents after each timeStep
double CA[numSensors*timeSteps*numStates]; // PTxn
double YBu[numSensors*timeSteps];  // PTx1
double U[numInputs*timeSteps];    // MxT
double Y[numSensors*timeSteps];
double y[numSensors];
double x[numStates]; // state vector

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


