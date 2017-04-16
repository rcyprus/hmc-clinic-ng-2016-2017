/* Produced by CVXGEN, 2017-03-26 23:17:32 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#include "CVX_inputs.h"
#include "solver.h"
#include <time.h>

Vars vars;
Params params;
Workspace work;
Settings settings;
//#define NUMTESTS 0

// Add declarations for load_CA and load_YBu
void load_CA(int timeStep);
void load_YBu(int timeStep, double* yin, double* Uin);

int main(int argc, char **argv) {
  int num_iters;
/*
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
*/
  set_defaults();
  setup_indexing();

  /* Solve problem instance for the record. */
  settings.verbose = 1;

  // Setup system matrices
  const char* filenameA = "Amatrix.txt";
  const char* filenameB = "Bmatrix.txt";
  const char* filenameC = "Cmatrix.txt";
  readArrayFromFile(filenameA, A);
  readArrayFromFile(filenameB, B);
  readArrayFromFile(filenameC, C);
  
  // Create string to store y filename
  const char* filenameY = "Ymatrix.txt";
  const char* filenameU = "Umatrix.txt";
  char filenameX[6];
  
  // START TIMING
  clock_t begin = clock();
  
  // *** CHANGE THIS TO A WHILE LOOP IN FINAL VERSION ***
  const size_t TT = 20;
  for (size_t ts = 0; ts < TT; ++ts) {
    
    // Read in matrix of most recent control inputs
    readLines(filenameU, U, ts, M, T);
    printArrayDouble(U,M,T);
    
    // In this case we only need to update the YBu matrix, not recreate it
    if (ts < T) {
      // Read in sensor data from file
      readLines(filenameY, y, ts, P, 1);
      //printArrayDouble(y,1,P);
      
      // Update solver parameters CA and YBu
      load_CA(ts);
      load_YBu(ts, y, U);
    }
    
    // Here we need to recreate the entire YBu matrix
    else {
      printf("else case load data\n");
      // Loop through T timesteps
      for (size_t t = 0; t < T; ++t) {
        int tstart = ts-(T-1);
        readLines(filenameY, y, tstart+t, P, 1);
        //printArrayDouble(y,1,P);
        
        // Create solver parameter YBu
        // (CA is already fully populated and does not change)
        load_YBu(t, y, U);
      }
    }

    // Print solver parameters
    //printf("Full CA matrix is (non-zero entries by T):\n");
    //printArrayDouble(params.CA, nonZeroEntries, T);
    //printf("Full YBu matrix is (P x T):\n");
    //printArrayDouble(params.YBu, P, T);
    
    // Solve optimization problem for initial state
    num_iters = solve();
    
    printf("\nOptimized x BEFORE dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);
    
    if (ts < T) {
      propagateDynamics(ts, U, vars.x);
    }
    else {
      printf("else case propagate dynamics\n");
      propagateDynamics(T, U, vars.x);
    }

    printf("Optimized x AFTER dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);

    // save output state x
    sprintf(filenameX, "x%u.txt", ts);
    writeFile(filenameX, vars.x, N);
  }

  // END TIMING
  clock_t end = clock();
  double time_spent = (double) (end - begin) / CLOCKS_PER_SEC * 1000;
  printf("Time to solve per timestep is is %.2f ms\n", time_spent/TT);

/*
  printf("A is:\n");
  printArrayDouble(A,N,N);
  printf("B is:\n");
  printArrayDouble(B,N,M);
  printf("C is:\n");
  printArrayDouble(C,P,N);
*/
/*
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  // Now solve multiple problem instances for timing purposes.
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
*/
  return 0;
}
void load_CA(int timeStep) {
  // Set up CVX parameter matrix CA
  updateCA(timeStep);
  for (size_t i = 0; i < nonZeroEntries*T; ++i) {
    params.CA[i] = CA[i];
  }
  //printArrayDouble(params.CA,nonZeroEntries,T);
}
void load_YBu(int timeStep, double* yin, double* Uin) {
  // Set up CVX parameter vector YBu
  updateYBu(timeStep, yin, Uin);
  for (size_t i = 0; i < P*T; ++i) {
    params.YBu[i] = YBu[i];
  }
  //printArrayDouble(params.YBu,P,T);
}
