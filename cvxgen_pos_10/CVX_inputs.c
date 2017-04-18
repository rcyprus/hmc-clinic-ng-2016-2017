// Inputs to quadrotor CVX optimization solver 

#include "CVX_inputs.h"
#include <stdio.h>

////////////////////////////
// CVXgen input functions //
////////////////////////////

// CVXgen sparse indicies
static int rowInd[nonZeroEntries] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 1, 2};
static int colInd[nonZeroEntries] = {1, 1, 2, 3, 4, 2, 3, 4, 2,  3,  4,  2,  3,  4, 4, 4};

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 * Updates the specified "section" of CA matrix (global)
 */
void updateCA(int timeStep) {
  // Initialize intermediate array and output array
  double tmpCA[numSensors*numStates];
  double AT[numStates*numStates];

  // Compute C*(A^T)
  power(timeStep, AT);
  multiply(C, numSensors, numStates, AT, numStates, numStates, tmpCA);

  //printf("tmpCA is:\n");
  //printArrayDouble(tmpCA,P,N);

  // Copy tmpCA into full CA matrix (no sparsity)
  /*
  for (int i = 0; i < N; ++i) {
    for (int p = 0; p < P; ++p) {
    //CA[(P*N)*timeStep+i] = tmpCA[i];
      CA[i*P*T + timeStep*P + p] = tmpCA[i*P + p];
    }
  }
  */
  int r = 0;
  int c = 0;
  for (int i = 0; i < nonZeroEntries; ++i) {
    r = rowInd[i]-1; // Matlab/CVXgen indexes from 1
    c = colInd[i]-1;
    CA[i + nonZeroEntries*timeStep] = tmpCA[c*numSensors + r];
  }

  // debugging
  //printf("CA is:\n");
  //printArrayDouble(CA,P*T,N);
}

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 * Inputs: timeStep: only updating at a given timeStep
 *         y: sensor outputs at current timeStep (Px1)
 *         U: control inputs (all time) (mTx1)
 * Output: YBu (global matrix) (PTx1)
 */
void updateYBu(int timeStep, double* yin, double* Uin) {
  // Initialize temporary arrays
  double AT[numStates*numStates];
  double CA[numSensors*numStates];
  double u[numInputs]; 
  double Bu[numStates];
  double CABu[numSensors*timeSteps];
  double tmpYBu[numSensors];
  double tmpYBu2[numSensors];

  // put sensor outputs into output matrix
  for (size_t i = 0; i < numSensors; ++i) {
    tmpYBu[i] = yin[i];
  }
  
  // Sums previous inputs up through current timeStep
  for (size_t i = 0; i < timeStep; ++i) {
    // Grab necessary section of U
    for (size_t j = 0; j < numInputs; ++j) {
      u[j] = Uin[ i*numInputs + j];
    }

    //printf("Timestep is %d\n",i);
    //printArrayDouble(B,n,m);
    //printArrayDouble(u,m,1);

    // Perform calculations
    power((timeStep-1)-i, AT);
    //printArrayDouble(AT,n,n);
    multiply(C,  numSensors, numStates, AT, numStates, numStates, CA);
    //printArrayDouble(CA,P,n);
    multiply(B,  numStates, numInputs, u, numInputs, 1, Bu);
    //printArrayDouble(Bu,n,1);
    multiply(CA, numSensors, numStates, Bu, numStates, 1, CABu);
    //printArrayDouble(CABu,P,1);
    sub(tmpYBu, CABu, numSensors, tmpYBu2);
    //printArrayDouble(tmpYBu,P,1);
    
    for (size_t j = 0; j < numSensors; ++j) {
      tmpYBu[j] = tmpYBu2[j];
    }
  }
  
  // Copy tmpYBu into full YBu matrix
  for (size_t i = 0; i < numSensors; ++i) {
    YBu[numSensors*timeStep+i] += tmpYBu[i];
  }
}

/* 
 * Given the initial state output from CVX and U (vector of global inputs),
 * propagate system dynamics to obtain the previous system state
 */
void propagateDynamics(int timeStep, double* Uin, double* x) {
  // Initialize temporary variables
  double u[numInputs];
  double Ax[numStates];
  double Bu[numStates];
  
  // Loop through timesteps
  for (size_t t = 0; t < timeStep; ++t) {
    // Grab necessary inputs
    for (size_t j = 0; j < numInputs; ++j) {
      u[j] = Uin[numInputs*t + j];
    }

    // Propagate system dynamics
    multiply(A, numStates, numStates, x, numStates, 1, Ax);
    multiply(B, numStates, numInputs, u, numInputs, 1, Bu);
    add(Ax, Bu, numStates, x);
  }
}

/* 
 * Raises the square matrix A with a size len x len to the power of t
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT) {
  // Create temporary matrix
  double tmpAT[numStates*numStates];

  // Fill the output AT matrix with zeros
  for (size_t i = 0; i < numStates*numStates; ++i) {
    AT[i] = 0;
  }
  
  // Make AT the identity matrix
  for (size_t i = 0; i < numStates; ++i) {
    AT[i*(numStates+1)] = 1;
  }
  
  // If t is zero we want to return the identity without doing multiplication
  if (t == 0) {
    return;
  }
  else {
    // Loop through the number of powers desired
    for (size_t i = 0; i < t; ++i) {
      multiply(A, numStates, numStates, AT, numStates, numStates, tmpAT);
      // Copy tmpAT into AT
      for (size_t j = 0; j < numStates*numStates; ++j) {
        AT[j] = tmpAT[j];
      }
    }
  }
}


