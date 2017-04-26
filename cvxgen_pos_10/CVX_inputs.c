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
  double tmpCA[P*N];
  double AT[N*N];

  // Compute C*(A^T)
  power(timeStep, AT);
  multiply(C, P, N, AT, N, N, tmpCA);

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
  int i = 0;
  for (i = 0; i < nonZeroEntries; ++i) {
    r = rowInd[i]-1; // Matlab/CVXgen indexes from 1
    c = colInd[i]-1;
    CA[i + nonZeroEntries*timeStep] = tmpCA[c*P + r];
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
  double AT[N*N];
  double CA[P*N];
  double u[M]; 
  double Bu[N];
  double CABu[P*T];
  double tmpYBu[P];
  double tmpYBu2[P];

  size_t i;
  size_t j;
  // put sensor outputs into output matrix
  for (i = 0; i < P; ++i) {
    tmpYBu[i] = yin[i];
  }
  
  // Sums previous inputs up through current timeStep
  for (i = 0; i < timeStep; ++i) {
    // Grab necessary section of U
    for (j = 0; j < M; ++j) {
      u[j] = Uin[ i*M + j];
    }

    //printf("Timestep is %d\n",i);
    //printArrayDouble(B,n,m);
    //printArrayDouble(u,m,1);

    // Perform calculations
    power((timeStep-1)-i, AT);
    //printArrayDouble(AT,n,n);
    multiply(C,  P, N, AT, N, N, CA);
    //printArrayDouble(CA,P,n);
    multiply(B,  N, M, u, M, 1, Bu);
    //printArrayDouble(Bu,n,1);
    multiply(CA, P, N, Bu, N, 1, CABu);
    //printArrayDouble(CABu,P,1);
    sub(tmpYBu, CABu, P, tmpYBu2);
    //printArrayDouble(tmpYBu,P,1);
    
    for (j = 0; j < P; ++j) {
      tmpYBu[j] = tmpYBu2[j];
    }
  }
  
  // Copy tmpYBu into full YBu matrix
  for (i = 0; i < P; ++i) {
    YBu[P*timeStep+i] += tmpYBu[i];
  }
}

/* 
 * Given the initial state output from CVX and U (vector of global inputs),
 * propagate system dynamics to obtain the previous system state
 */
void propagateDynamics(int timeStep, double* Uin, double* x) {
  // Initialize temporary variables
  double u[M];
  double Ax[N];
  double Bu[N];
  
  size_t t;
  size_t j;
  // Loop through T
  for (t = 0; t < timeStep; ++t) {
    // Grab necessary inputs
    for (j = 0; j < M; ++j) {
      u[j] = Uin[M*t + j];
    }

    // Propagate system dynamics
    multiply(A, N, N, x, N, 1, Ax);
    multiply(B, N, M, u, M, 1, Bu);
    add(Ax, Bu, N, x);
  }
}

/* 
 * Raises the square matrix A with a size len x len to the power of t
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT) {
  // Create temporary matrix
  double tmpAT[N*N];

  size_t i;
  size_t j;
  // Fill the output AT matrix with zeros
  for (i = 0; i < N*N; ++i) {
    AT[i] = 0;
  }
  
  // Make AT the identity matrix
  for (i = 0; i < N; ++i) {
    AT[i*(N+1)] = 1;
  }
  
  // If t is zero we want to return the identity without doing multiplication
  if (t == 0) {
    return;
  }
  else {
    // Loop through the number of powers desired
    for (i = 0; i < t; ++i) {
      multiply(A, N, N, AT, N, N, tmpAT);
      // Copy tmpAT into AT
      for (j = 0; j < N*N; ++j) {
        AT[j] = tmpAT[j];
      }
    }
  }
}


