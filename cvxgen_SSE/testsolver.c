/* Produced by CVXGEN, 2017-02-21 01:47:44 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
#include "inputs.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
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
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    char* filename = sprintf("smally%d.txt", i);
    readArrayFromFile(filename, y);
    load_data(i, y, U);
    solve();

    // propagate system dynamics on initial state
    propagateDynamics(i, U, vars.x);

    // print optimization result for x
    for (int i = 0; i < 5; i++)
      printf("  %9.4f\n", vars.x[i]);
    return 0;
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
}
void load_data(int timeStep, double* y, double* U) {
  // Set up CVX parameters
  setupI();
  for (int p = 0; p < P; ++p) {
    for (size_t i = 0; i < T*T*P; ++i) {
      params.Ip[p][i] = I[p][i];
    }
  }
  
  updateCA(timeStep);
  for (size_t i = 0; i < P*T*n; ++i) {
    params.CA[i] = CA[i];
  }

  updateYBu(timeStep, y, U);
  for (size_t i = 0; i < P*T; ++i) {
    params.YBu[i] = YBu[i];
  }
  
}
