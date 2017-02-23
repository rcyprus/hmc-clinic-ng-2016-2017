// Inputs to quadrotor CVX optimization solver 

#include "inputs.h"

// Define constants
int n = 10; // states
int P = 15; // sensors
int T = 3; // timesteps
int m = 5; // inputs

// TODO: Define system matrices from a text file??!

/* 
 * Creates a T x PT matrix for with the indexed variable p
 */
void createI(int T, int P, double* I) {
  // Loop through each indexed variable
  for (size_t p = 0; p < P; ++p) {
    
    // Loop through rows
    for (size_t i = 0; i < T; ++i) {
      // Loop through columns
      for (size_t j = 0; j < P*T; ++j) {
        // Populate the matrix with zeros
        I[p+1][i + j*T] = 0;
      }
    }
    
    // Put ones where we want them
    for (size_t t = 0; t < T; ++t) {
      I[p+1][p + t*(P+P*T)] = 1;
    }
  }
}

/* 
 * Creates a PT x n matrix based on matrices A and C of quadrotor model
 */
void createCA(int T, int P, int n, double* A, double* C, double* CA) {
  // Initialize intermediate array and output array
  double tmpCA = double[P*n];
  double* CA = double[(P*T)*n];
  
  // Loop through each timestep
  for (size_t t = 0; t < T; ++t) {
    // Compute C*(A^T)
    power(A, n, T, AT);
    multiply(C, P, n, AT, n, n, tmpCA);

    // Copy tmpCA into full CA matrix
    for (size_t i = 0; i < P*n; ++i) {
      CA[(P*n)*t+i] = tmpCA[i];
    }
  }
}

/* 
 * Creates a PT x 1 matrix of sensor measurements and propagated inputs
 */
void createYBu(double* A, double* B, double* C,
               int T, int P, int n, int m,
               double* Y, double* U, double* YBu) {
  // Initialize temporary arrays
  double[] u = double[m];
  double[] CA = double[(P*T)*n];
  double[] Bu = double[n];
  double[] CABu = double[P*T];

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
      add(Y, -YBu, P*T, YBu);
    }
  }
}

/* 
 * Raises the square matrix A with a size len x len to the power of T
 */
void power(double* A, int len, int T, double* AT) {
  // Initialize output matrix
  double* AT = double[len*len];
  
  // If T is zero we don't need to multiply anything
  if (T == 0) {
    AT = A;
  }
  else {
    // Loop through the number of powers desired
    for (size_t i = 0; i < T, ++i) {
      multiply(A, len, len, A, len, len, AT);
    }
  }
}

/* 
 * Multiplies together matrices A and B (sizes must be compatible)
 */
void multiply(double* A, int rowsA, int colsA,
              double* B, int rowsB, int colsB,
              double* AB) {
  // Throw an exception if the matrix sizes are not compatible
  if (colsA != rows B) {
    // TODO: throw an exception
  }

  // Initialize the output matrix to be the correct size and fill it with zeros
  double* AB = double[rowsA*colsB];
  for (size_t i = 0; i < rowsA*colsB; ++i) {
    AB[i] = 0;
  }

  // Define temporary arrays
  double[] tmprowA = double[colsA];
  double[] tmpcolB = double[rowsB];
  
  for (size_t c = 0; c < colsB; ++c) { // 4 times
    for (size_t r = 0; r < rowsA; ++r) { // 3 times
      // Fill temporary arrays
      for (size_t i = 0; i < colsA; ++i) {
        tmprowA[i] = A[(r*rowsA)+i];
      }
      for (size_t i = 0; i < rowsB; ++i) {
        tmprowB[i] = B[c+(i*colsB)];
      }
      
      // Save dot product in output array
      AB[c + r*colsB] = dot(tmprowA, tmpcolB, colsA);
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


