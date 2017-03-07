// Inputs to quadrotor CVX optimization solver 

#include "inputs.h"
#include <stdio.h>

////////////////////////////
// CVXgen input functions //
////////////////////////////
/* 
 * Setup a P, TxPT matrices for with the indexed variable p
 */
void setupI(void) {
  // Loop through each indexed variable
  for (int p = 0; p < P; ++p) {
    // Put ones where we want them
    for (int t = 0; t < T; ++t) {
      //I[p+1][p + t*(P+P*T)] = 1;
      //I[p+1][p + t*P + t*P*T] = 1;
      I[p+1][t*P*T + t + p*T] = 1;
    }
  }
}

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

  // Copy tmpCA into full CA matrix
  for (int i = 0; i < P*N; ++i) {
    CA[(P*N)*timeStep+i] = tmpCA[i];
  }
}

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 * Inputs: timeStep: only updating at a given timeStep
 *         y: sensor outputs at current timeStep (Px1)
 *         U: control inputs (all time) (mTx1)
 * Output: YBu (global matrix) (PTx1)
 */
void updateYBu(int timeStep, double* y, double* U) {
  // Initialize temporary arrays
  double AT[N*N];
  double CA[P*N];
  double u[M]; 
  double Bu[N];
  double CABu[P*T];
  double tmpYBu[P];
  double tmpYBu2[P];

  // put sensor outputs into output matrix
  for (size_t i = 0; i < P; ++i) {
    tmpYBu[i] = y[i];
    //YBu[P*timeStep+i] = y[i];
  }
  
  // Sums previous inputs up through current timeStep
  for (size_t i = 0; i < timeStep; ++i) {
    // Grab necessary section of U
    for (size_t j = 0; j < M; ++j) {
      u[j] = U[ i*M + j];
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
    
    for (size_t j = 0; j < P; ++j) {
      tmpYBu[j] = tmpYBu2[j];
    }
  }
  // Copy tmpYBu into full YBu matrix
  for (size_t i = 0; i < P; ++i) {
    YBu[P*timeStep+i] += tmpYBu[i];
  }
}

/* 
 * Given the initial state output from CVX and U (vector of global inputs),
 * propagate system dynamics to obtain the previous system state
 */
void propagateDynamics(int timeStep, double* U, double* x) {
  // Initialize temporary variables
  double u[M];
  double Ax[N];
  double Bu[N];
  
  // Loop through timesteps
  for (size_t t = 0; t < timeStep; ++t) {
    // Grab necessary inputs
    for (size_t j = 0; j < M; ++j) {
      u[j] = U[M*t+j];
    }

    // Propagate system dynamics
    multiply(A, N, N, x, N, 1, Ax);
    multiply(B, N, M, u, M, 1, Bu);
    add(Ax, Bu, N, x);
  }
}

/////////////////////////////
// Matrix helper functions //
/////////////////////////////
/* 
 * Raises the square matrix A with a size len x len to the power of t
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT) {
  // Create temporary matrix
  double tmpAT[N*N];

  // Fill the output AT matrix with zeros
  for (size_t i = 0; i < N*N; ++i) {
    AT[i] = 0;
  }
  
  // Make AT the identity matrix
  for (size_t i = 0; i < N; ++i) {
    AT[i*(N+1)] = 1;
  }
  
  // If t is zero we want to return the identity without doing multiplication
  if (t == 0) {
    return;
  }
  else {
    // Loop through the number of powers desired
    for (size_t i = 0; i < t; ++i) {
      multiply(A, N, N, AT, N, N, tmpAT);
      // Copy tmpAT into AT
      for (size_t j = 0; j < N*N; ++j) {
        AT[j] = tmpAT[j];
      }
    }
  }
}

/* 
 * Multiplies together matrices X and Y (sizes must be compatible)
 * and saves it into input XY
 */
void multiply(double* X, int rowsX, int colsX,
              double* Y, int rowsY, int colsY,
              double* XY ) {
  // Throw an exception if the matrix sizes are not compatible
  if (colsX != rowsY) {
    printf("ERROR: Matrix dimensions must match");
  }

  // Initialize an output matrix of the correct size and fill with zeros
  for (int i = 0; i < rowsX*colsY; ++i) {
    XY[i] = 0;
  }

  // Define temporary arrays
  double tmprowX[colsX];
  double tmpcolY[rowsY];
  
  for (int r = 0; r < rowsX; ++r) { // 3 times

    // grab the correct row of the first matrix
    for (int i = 0; i < colsX; ++i) {
      tmprowX[i] = X[(r*colsX)+i];
    }

    for (int c = 0; c < colsY; ++c) { // 4 times

      // grab the correct column of the second matrix
      for (int j = 0; j < rowsY; ++j) {
        tmpcolY[j] = Y[c+(j*colsY)];
      }

      //printArrayDouble(tmprowX,colsX,1);
      //printArrayDouble(tmpcolY,rowsY,1);

      // Save dot product in output array
      //XY[c + r*colsY] = dot(tmprowX, tmpcolY, colsX);
      XY[c*rowsX + r] = dot(tmprowX, tmpcolY, colsX);
      //printf("Dot product is: %lf\n",XY[c + r*colsY]);
    }
  }
}

///////////////////////////////////
// Scalar array helper functions //
///////////////////////////////////
/* 
 * Takes the dot product of vectors x and y (vectors must have length len)
 */
double dot(double* x, double* y, int len) {
  // Initialize output to be zero
  double xy = 0;
  
  // Loop through the length of the vector and take the dot product
  for (int i = 0; i < len; ++i) {
    xy += x[i]*y[i];
  }
  
  // return the final value
  return xy;
}

/* 
 * Adds vectors x and y together (both vectors must of of length len)
 */
void add(double* x, double* y, int len, double* xplusy) {
  // Loop through the two vectors and add elements
  for (int i = 0; i < len; ++i) {
    xplusy[i] = x[i] + y[i];
  }
}
void sub(double* x, double* y, int len, double* xminy) {
  // Loop through the two vectors and add elements
  for (int i = 0; i < len; ++i) {
    xminy[i] = x[i] - y[i];
  }
}

/////////////////////
// Print functions //
/////////////////////
/*
 * Debugging print function (only 2D double arrays)
 */
void printArrayDouble(double* array, int rows, int cols){
  // Loop down rows
  for(int j = 0; j < cols; ++j){
  //for(int i = 0; i < rows; ++i){
    printf("  ");
    // Loop across row
    for(int i = 0; i < rows; ++i){
    //for(int j = 0; j < cols; ++j){
      //printf("%lf ", array[i*cols + j]);
      printf("%lf ", array[i + j*cols]);
    }
    // Print enter
    printf("\n");
  }
  printf("\n");
}

/*
 * Debugging print function (only 2D int arrays)
 */
void printArrayInt(int* array, int rows, int cols){
  // Loop down rows
  for(int j = 0; j < cols; ++j){
  //for(int i = 0; i < rows; ++i){
    printf("  ");
    // Loop across row
    for(int i = 0; i < rows; ++i){
    //for(int j = 0; j < cols; ++j){
      //printf("%lf ", array[i*cols + j]);
      printf("%d ", array[i + j*cols]);
    }
    // Print enter
    printf("\n");
  }
  printf("\n");
}

///////////////////////////////////
// Read in arrays from text file //
///////////////////////////////////
/*
 * Read in 1D array from file (w/ comma delimiter) 
 */
void readArrayFromFile(const char* file_name, double* array){
  // Open file
  FILE* file = fopen (file_name, "r");
  
  // Check if file was opened
  if(file == NULL){
    fprintf(stderr,"Failed to open file '%s'\n",file_name);
  }

  printf("Scanning file: %s\n", file_name);

  double i = 0;
  int index = 0;

  // Read until end of file
  while(fscanf(file, "%lf,", &i) > 0)
  {  
      array[index] = i;
      index++; 
      // printf ("%lf ", i);
  }
  // Close file
  fclose (file);        
}

// Matlab
/*
  A = rand(N); //square matrix
  A = reshape(A',[1,N*N]);
  dlmwrite('filename.txt',A);
*/


