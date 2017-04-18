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

// Add declarations for load_CA and load_YBu
void load_CA(int timeStep);
void load_YBu(int timeStep, double* yin, double* Uin);

int main(int argc, char **argv) {
  // CVX setup
  set_defaults();
  setup_indexing();
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
    printf("Current timestep is: %d\n", ts);

    if (flag) {
      // In this case we only need to update the YBu matrix, not recreate it
      if (ts < T) {
        // Read in matrix of most recent control inputs
        readLines(filenameU, U, 0, M, T);
        printArrayDouble(U,M,T);

        // Read in sensor data from file
        readLines(filenameY, y, ts, P, 1);
        //printArrayDouble(y,1,P);
        
        // Update solver parameters CA and YBu
        load_CA(ts);
        load_YBu(ts, y, U);
      }
      
      // Here we need to recreate the entire YBu matrix
      else {
        printf("Windowing load data\n");
        
        int tstart = ts-(T-1);

        // clear old data in YBu
        for (size_t t = 0; t < P*T; ++t) {
          YBu[t] = 0;
        }

        // Read in matrix of most recent control inputs
        readLines(filenameU, U, tstart, M, T);
        printArrayDouble(U,M,T);

        // Loop through T timesteps
        for (size_t t = 0; t < T; ++t) {
          
          readLines(filenameY, y, tstart+t, P, 1);
          printArrayDouble(y,1,P);

          // Create solver parameter YBu
          // (CA is already fully populated and does not change)
          load_YBu(t, y, U);
        }

        printf("Full YBu matrix is (P x T):\n");
        printArrayDouble(params.YBu, P, T);
      }
      
      // Print solver parameters
      //printf("Full CA matrix is (non-zero entries by T):\n");
      //printArrayDouble(params.CA, nonZeroEntries, T);
      //printf("Full YBu matrix is (P x T):\n");
      //printArrayDouble(params.YBu, P, T);
      
      // Solve optimization problem for initial state
      solve();
      
      printf("\nOptimized x BEFORE dynamics propagation is:\n");
      printArrayDouble(vars.x, N, 1);

      for (size_t i = 0; i < N; ++i) {
        x[i] = vars.x[i];
      }
      
      if (ts < T) {
        propagateDynamics(ts, U, x);
      }
      else {
        printf("Windowing propagate dynamics\n");
        propagateDynamics(T-1, U, x);
      }
      
      printf("Optimized x AFTER dynamics propagation is:\n");
      printArrayDouble(x, N, 1);
      
      // save output state x
      sprintf(filenameX, "x%u.txt", ts);
      writeFile(filenameX, x, N);
    }
    else {
      dt;

      // Copy old state x into x_prev
      for (size_t i = 0; i < N; ++i) {
        x_prev[i] = x[i];
      }

      // Approximate angular accelerations and calculate new orientation/
      // angular velocities
      int alphax = (x_prev[7] - x_pprev[7])/dt;
      int alphay = (x_prev[8] - x_pprev[8])/dt;
      int alphaz = (x_prev[9] - x_pprev[9])/dt;
      x[4] = x_prev[4] + x_prev[7]*dt + 0.5*alphax*dt*dt;
      x[5] = x_prev[5] + x_prev[8]*dt + 0.5*alphay*dt*dt;
      x[6] = x_prev[6] + x_prev[9]*dt + 0.5*alphaz*dt*dt;
      x[7] = x_prev[7] + alphax*dt;
      x[8] = x_prev[8] + alphay*dt;
      x[9] = x_prev[9] + alphaz*dt;

      // Copy former x_prev into x_pprev
      for (size_t i = 0; i < N; ++i) {
        x_pprev[i] = x_prev[i];
      }

    }
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
