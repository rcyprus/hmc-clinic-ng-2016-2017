// Inputs to quadrotor CVX optimization solver 

#include "inputs.h"
#include <stdio.h>

// Define constants
// const int n = 10; // states
// const int P = 38; // sensors
// const int T = 100; // timesteps
// const int m = 5; // inputs
#define n 10
#define P 38
#define T 100
#define m 5

// Constant Matrices
// // TODO: Define system matrices from a text file??!
const double A[n*n]; // nxn
const double B[n*m]; // nxm
const double C[P*n]; // Pxn

// Constant (but need to populate)
double I[P+1][T*T*P] = {0}; // P+1 b/c CVXGEN may use only the latter 5 rows ...

// Update contents after each timeStep
double CA[P*T*n]; // PTxn
double YBu[P*T]; // PTx1


/* 
 * Setup a P, TxPT matrices for with the indexed variable p
 */
void setupI(void) {
  // Loop through each indexed variable
  for (int p = 0; p < P; ++p) {
    // Put ones where we want them
    for (int t = 0; t < T; ++t) {
      I[p+1][p + t*(P+P*T)] = 1;
    }
  }
}

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 * Updates the specified "section" of CA matrix (global)
 */
void updateCA(int timeStep) {
  // Initialize intermediate array and output array
  double tmpCA[P*n];
  double AT[n*n];

  // Compute C*(A^T)
  power(timeStep, AT);
  multiply(C, P, n, AT, n, n, tmpCA);

  // Copy tmpCA into full CA matrix
  for (int i = 0; i < P*n; ++i) {
    CA[(P*n)*timeStep+i] = tmpCA[i];
  }
}

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 * Inputs: timeStep: only updating at a given timeStep
 *         y: sensor outputs at current timeStep (Px1)
 *         U: control inputs (all time) (mTx1)
 * Output: YBu (global matrix) (PTx1)
 */
void updateYBu(int timeStep, double* y, double* U, double* YBu) {
  // Initialize temporary arrays
  double tmpCA[P*n];
  double u[m]; 
  double Bu[n];
  double CABu[P*T];
  double AT[n*n];
  double tmpYBu[P];
  
  // Sums previous inputs up through current timeStep
  for (int i = 1; i < timeStep; ++i) {
    // Grab necessary section of U
    for (int j = 0; j < m; ++j) {
      u[j] = U[ ((i-1)*m) + j];
    }
    
    // Perform calculations
    power(timeStep-i, AT);
    multiply(C, P, n, AT, n, n, tmpCA);
    multiply(B, n, m, u, m, 1, Bu);
    multiply(CA, P, n, Bu, n, 1, CABu);
    sub(y, CABu, P*T, tmpYBu);
  }

  // Copy tmpYBu into full YBu matrix
  for (int i = 0; i < P; ++i) {
    YBu[P*timeStep+i] = tmpYBu[i];
  }
}

/* 
 * Raises the square matrix A with a size len x len to the power of T
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT) {
  // Initialize output matrix
  //double temp[n*n];
  
  // If T is zero we don't need to multiply anything
  if (t == 0) {
    AT = A;
  }
  else {
    // Loop through the number of powers desired
    for (int i = 1; i < t; ++i) {
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
  for (int i = 0; i < rowsX*colsY; ++i) {
    XY[i] = 0;
  }

  // Define temporary arrays
  double tmprowX[colsX];
  double tmpcolY[rowsY];
  
  for (int r = 0; r < rowsX; ++r) { // 3 times
    for (int c = 0; c < colsY; ++c) { // 4 times
      // Fill temporary arrays
      for (int i = 0; i < colsX; ++i) {
        tmprowX[i] = X[(r*colsX)+i];
      }
      for (int i = 0; i < rowsY; ++i) {
        tmpcolY[i] = Y[c+(i*rowsY)];
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


/*
 * Debugging print function (only 2D arrays)
 */
void printArray(double* array, int rows, int cols){
  // Loop down rows
  for(int i = 0; i < rows; ++i){
    // Loop across row
    for(int j = 0; j < cols; ++j){
      printf("%lf ", array[i*cols + j]);
    }
    // Print enter
    printf("\n");
  }
}

/*
 * Read in 1D array from file (w/ comma delimiter) 
 */
void readArrayFromFile(const char* file_name, double* array){
  FILE* file = fopen (file_name, "r");
  double i = 0;
  int index = 0;

  fscanf (file, "%lf", &i); 
  while (!feof (file))
    {  
      array[index] = i;
      index++; 
      // printf ("%d ", i);
      fscanf (file, "%lf,", &i);
    }
  fclose (file);        
}

// Matlab
/*
  A = rand(N); //square matrix
  A = reshape(A',[1,N*N]);
  dlmwrite('filename.txt',A);

*/

int main(void){
  printf("compiled\n");
  return 0;
}
