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
#define T 4
#define m 5

// Constant Matrices
// // TODO: Define system matrices from a text file??!
double A[n*n]; // nxn
double B[n*m]; // nxm
double C[P*n]; // Pxn

// Constant (but need to populate)
int I[P+1][T*T*P]; // P+1 b/c CVXGEN may use only the latter 5 rows ...

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
 * Raises the square matrix A with a size len x len to the power of t
 * (Currently only applied to global matrix A)
 */
void power(int t, double* AT) {
  // Create temporary matrix
  double tmpAT[n*n];

  // Fill the output AT matrix with zeros
  for (size_t i = 0; i < n*n; ++i) {
    AT[i] = 0;
  }
  
  // Make AT the identity matrix
  for (size_t i = 0; i < n; ++i) {
    AT[i*(n+1)] = 1;
  }
  
  // If t is zero we want to return the identity without doing multiplication
  if (t == 0) {
    return;
  }
  else {
    // Loop through the number of powers desired
    for (size_t i = 0; i < t; ++i) {
      multiply(A, n, n, AT, n, n, tmpAT);
      // Copy tmpAT into AT
      for (size_t j=0; j < n*n; ++j) {
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
    // TODO
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
    printf("  ");
    // Loop across row
    for(int j = 0; j < cols; ++j){
      printf("%lf ", array[i*cols + j]);
    }
    // Print enter
    printf("\n");
  }
}

/*
 * Debugging print function (only 2D int arrays)
 */
void printArrayInt(int* array, int rows, int cols){
  // Loop down rows
  for(int i = 0; i < rows; ++i){
    printf("  ");
    // Loop across row
    for(int j = 0; j < cols; ++j){
      printf("%d ", array[i*cols + j]);
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
  // printf("break 1");
  double i = 0;
  int index = 0;
  printf("Scanning file: %s\n", file_name);
  fscanf (file, "%lf,", &i); 
  
  // printf("break 2");
  
  while (!feof (file))
  {  
      array[index] = i;
      index++; 
      // printf ("%d ", i);
      fscanf (file, "%lf,", &i);
  }
  array[index] = i;
  fclose (file);        
}

// Matlab
/*
  A = rand(N); //square matrix
  A = reshape(A',[1,N*N]);
  dlmwrite('filename.txt',A);

*/
void simpleFile(void)
{
    FILE* f = fopen("test.txt", "r");
    int k = 0;
    int i = 0;
    int numbers[10]; // assuming there are only 5 numbers in the file

    while( fscanf(f, "%d,", &k) > 0 ) // parse %d followed by ','
    {
        numbers[i++] = k;
    }

    for(int j = 0; j < 10; j++){
      printf("%d, ",numbers[j]);
    }
    printf("\n");

    fclose(f);
}


int main(void){
  printf("compiled\n");
  // printf("try load file\n");
  // Initialize test array
  double test[25];
  
  // Fill Test array with text file and print
  readArrayFromFile("test.txt", test);
  printArray(test,5,5);
  
  // Test dot product
  double testDot = dot(test,test,25);
  printf("Test dot: %lf\n",testDot);
  
  // Test multiply
  double testMultiply[25];
  multiply(test,5,5, test,5,5, testMultiply);
  printf("Test muliply: \n");
  printArray(testMultiply, 5,5);
  
  // Test I matrix
  setupI();
  printf("Test I[1] matrix:\n");
  printArrayInt(I[1], T, T*P);
  printf("Test I[5] matrix:\n");
  printArrayInt(I[5], T, T*P);
  
  // Load A, B, and C matrices
  readArrayFromFile("Amatrix.txt", A);
  readArrayFromFile("Bmatrix.txt", B);
  readArrayFromFile("Cmatrix.txt", C);

  // Test CA matrix
  //updateCA(T);
  //printArray(CA, P*T, n);
  
  // Test power
  double AT[n*n];
  power(0, AT); // raise A to the zero power, should output the identity
  printArray(AT,n,n);
  printf("\n");
  power(1, AT); // raise A to the first power, should output A
  printArray(AT,n,n);
  printf("\n");
  power(2, AT); // raise A to the second power, should output A^2
  printArray(AT,n,n);
  
  return 0;
}
