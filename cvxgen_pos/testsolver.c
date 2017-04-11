/* Produced by CVXGEN, 2017-03-26 23:36:48 -0400.  */
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
#define NUMTESTS 0

// Add declaration for load_data because C is upset
void load_data(int timeStep, double* yin, double* Uin);

int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();

  /* Solve problem instance for the record. */
  settings.verbose = 1;

  // Setup system matrices
  char* filenameA = "Amatrix.txt";
  char* filenameB = "Bmatrix.txt";
  char* filenameC = "Cmatrix.txt";
  readArrayFromFile(filenameA, A);
  readArrayFromFile(filenameB, B);
  readArrayFromFile(filenameC, C);

  // Read in matrix of control inputs
  char* filenameU = "Uvector.txt";
  readArrayFromFile(filenameU, U);
  printf("\n");
  
  // Create string to store y filename
  char filenameY[6];
  char filenameX[6];

  // START TIMING
  clock_t begin = clock();
  
  for (size_t ts = 0; ts < T; ++ts) {
    
    // Read in sensor data from file
    sprintf(filenameY, "y%u.txt", ts);
    printf("filename Y is: %s\n", filenameY);
    readArrayFromFile(filenameY, y);
    //printArrayDouble(y,1,P);
    
    // Create solver parameters CA and YBu
    load_data(ts, y, U);
    
    // Print solver parameters
    printf("Full CA matrix is (non-zero entries by T):\n");
    printArrayDouble(params.CA, nonZeroEntries, T);
    printf("Full YBu matrix is (P x T):\n");
    printArrayDouble(params.YBu, P, T);
    
    // Solve optimization problem for initial state
    num_iters = solve();
    
    printf("\nOptimized x BEFORE dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);
    
    // propagate system dynamics on initial state
    propagateDynamics(ts, U, vars.x);
    
    printf("Optimized x AFTER dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);

    // save output state x
    sprintf(filenameX, "x%u.txt", ts);
    writeFile(filenameX, vars.x, N);
  }
  
  // END TIMING
  clock_t end = clock();
  double time_spent = (double)(end - begin) / CLOCKS_PER_SEC * 1000;
  printf("Time to solve per timestep is is %.2f ms\n", time_spent/T);

#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
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
  return 0;
}
void load_data(int timeStep, double* yin, double* Uin) {
  // Set up CVX parameter matrix CA
  updateCA(timeStep);
  for (size_t i = 0; i < nonZeroEntries*T; ++i) {
    params.CA[i] = CA[i];
  }
  //printArrayDouble(params.CA,nonZeroEntries,T);

  // Set up CVX parameter vector YBu
  updateYBu(timeStep, yin, Uin);
  for (size_t i = 0; i < P*T; ++i) {
    params.YBu[i] = YBu[i];
  }
  //printArrayDouble(params.YBu,P*T,1);
}
