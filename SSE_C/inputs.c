// Inputs to quadrotor CVX optimization solver 

#include "inputs.h"

// Define constants
const int n = 10; // states
const int P = 38; // sensors
const int T = 100; // timesteps
const int m = 5; // inputs

// Constant Matrices
// // TODO: Define system matrices from a text file??!
int A[n*n]; // nxn
int B[n*m]; // nxm
int C[P*n]; // Pxn

// Constant (but need to populate)
int I[P+1][T*T*P] = {0}; // P+1 b/c CVXGEN may use only the latter 5 rows ...


int CA[P*T*n]; // PTxn , might need to grow as we go
int YBu[]; // PTx1


/* 
 * Setup a P, TxPT matrices for with the indexed variable p
 */
void setupI(void) {
  // Loop through each indexed variable
  for (size_t p = 0; p < P; ++p) {
    // Put ones where we want them
    for (size_t t = 0; t < T; ++t) {
      I[p+1][p + t*(P+P*T)] = 1;
    }
  }
}

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 * Updates the specified "section" of CA matrix
 */
void updateCA(int timeStep) {
  // Initialize intermediate array and output array
  double tmpCA[P*n];
  
  double AT[n*n];

  // Loop through each timestep
  // for (size_t t = 0; t < T; ++t) {
    // Compute C*(A^T)
    power(A, n, timeStep, AT);
    multiply(C, P, n, AT, n, n, tmpCA);

    // Copy tmpCA into full CA matrix
    for (size_t i = 0; i < P*n; ++i) {
      CA[(P*n)*timeStep+i] = tmpCA[i];
    }
  // }
}

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 */
void createYBu(double* A, double* B, double* C,
               int T, int P, int n, int m,
               double* Y, double* U, double* YBu) {
  // Initialize temporary arrays
  double u[m];
  double CA[(P*T)*n];
  double Bu[n];
  double CABu[P*T];

  // Set all elements of CA to zero initially
  for (size_t i = 0; i < P*T; ++i) {
    CABu[i] = 0;
  }
  
  // Loop through timesteps
  for (size_t t = 0; t < T; ++t) {
    for (size_t i = 1; i < T; ++i) {
      // Grab necessary section of U
      for (size_t j = 0; j < m; ++j) {
        u[j] = U[(i*m)+j];
      }
      
      // Perform calculations
      power(A, n, t-i, AT);
      multiply(C, P, n, AT, n, n, CA);
      multiply(B, n, m, u, m, 1, Bu);
      multiply(CA, P, n, Bu, n, 1, CABu);
      add(Y, -CABu, P*T, YBu);
    }
  }
}

/* 
 * Raises the square matrix A with a size len x len to the power of T
 * (Currently only applied to global matrix A)
 */
//void power(double A, int len, int T, double* AT) {
void power(int t, double* AT) {
  // Initialize output matrix
  //double temp[n*n];
  
  // If T is zero we don't need to multiply anything
  if (t == 0) {
    AT = A;
  }
  else {
    // Loop through the number of powers desired
    for (size_t i = 1; i < t, ++i) {
      multiply(A, n, n, AT, n, n, AT);
    }
  }
}

/* 
 * Multiplies together matrices X and Y (sizes must be compatible), saves it into input XY
 */
void multiply(double* X, int rowsX, int colsX,
              double* Y, int rowsY, int colsY,
              double* XY ) {
  // Throw an exception if the matrix sizes are not compatible
  if (colsX != rowsY) {
    // TODO
    printf("ERROR: Matrix dimensions must match");
  }

  // Initialize the output matrix to be the correct size and fill it with zeros
  for (size_t i = 0; i < rowsX*colsY; ++i) {
    XY[i] = 0;
  }

  // Define temporary arrays
  double tmprowX[colsX];
  double tmpcolY[rowsY];
  
  for (size_t r = 0; r < rowsX; ++r) { // 3 times
    for (size_t c = 0; c < colsY; ++c) { // 4 times
      // Fill temporary arrays
      for (size_t i = 0; i < colsX; ++i) {
        tmprowX[i] = X[(r*colsX)+i];
      }
      for (size_t i = 0; i < rowsY; ++i) {
        tmprowY[i] = Y[c+(i*rowsY)];
      }
      
      // Save dot product in output array
      XY[c + r*colsY] = dot(tmprowX, tmpcolY, colsX);
    }
  }
}

/* 
 * Takes the dot product of vectors x and y (vectors must have length len)
 */
double dot(double* x, double* y, int len) {
  // Initialize output to be zero
  double xy = 0;
  
  // Loop through the length of the vector and take the dot product
  for (size_t i = 0; i < len; ++i) {
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
  for (size_t i = 0; i < len; ++i) {
    xplusy[i] = x[i] + y[i];
  }
}

/*
 * Debugging print function (only 2D arrays)
 */
void printArray(double* array, int rows, int cols){
  // Loop down rows
  for(int i = 0; i < rows; ++i){
    // Loop across row
    for(int j = 0; j < cols; ++j){
      printf('%d ', array[i*rows + j]);
    }
    // Print enter
    printf('\n');
  }
}

