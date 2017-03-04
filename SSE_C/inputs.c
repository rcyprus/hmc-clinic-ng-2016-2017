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
  double tmpYBu[P];
  double CA[P*n];
  double u[m]; 
  double Bu[n];
  double CABu[P*T];
  double AT[n*n];
  
  // at timestep zero no inputs are necessary
  if (timeStep == 0) {
    for (size_t i = 0; i < P; ++i) {
      tmpYBu[i] = y[i];
    }
  }
  
  // Sums previous inputs up through current timeStep
  for (size_t i = 1; i <= timeStep; ++i) {
    // Grab necessary section of U
    for (size_t j = 0; j < m; ++j) {
      u[j] = U[ (i-1)*m + j];
    }
    
    // Perform calculations
    power(timeStep-i, AT);
    multiply(C,  P, n, AT, n, n, CA);
    multiply(B,  n, m, u,  m, 1, Bu);
    multiply(CA, P, n, Bu, n, 1, CABu);
    sub(y, CABu, P, tmpYBu);
  }

  // Copy tmpYBu into full YBu matrix
  for (size_t i = 0; i < P; ++i) {
    YBu[P*timeStep+i] = tmpYBu[i];
  }
}

/* 
 * Given the initial state output from CVX and U (vector of global inputs),
 * propagates system dynamics to obtain the previous system state
 */
void propagateDynamics(int timeStep, double* U, double* x) {
  // Initialize temporary variables
  double u[m];
  double Ax[n];
  double Bu[n];

  // Loop through timesteps
  for (size_t t = 0; t < timeStep; ++t) {
    // Grab necessary inputs
    for (size_t j = 0; j < m; ++j) {
      u[j] = U[m*t+j];
    }

    // Propagate system dynamics
    multiply(A, n, n, x, n, 1, Ax);
    multiply(B, n, m, u, m, 1, Bu);
    add(Ax, Bu, n, x);
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
 * Debugging print function (only 2D double arrays)
 */
void printArrayDouble(double* array, int rows, int cols){
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
  printArrayDouble(test,5,5);
  
  // Test dot product
  double testDot = dot(test,test,25);
  printf("Test dot: %lf\n",testDot);
  
  // Test multiply
  double testMultiply[25];
  multiply(test,5,5, test,5,5, testMultiply);
  printf("Test muliply: \n");
  printArrayDouble(testMultiply, 5,5);
  
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

  // Test power
  double AT[n*n];
  power(0, AT); // raise A to the zero power, should output the identity
  //printArrayDouble(AT,n,n);
  printf("\n");
  power(1, AT); // raise A to the first power, should output A
  //printArrayDouble(AT,n,n);
  printf("\n");
  power(2, AT); // raise A to the second power, should output A^2
  //printArrayDouble(AT,n,n);
  
  // Test CA matrix
  printf("\n");
  updateCA(0);
  updateCA(1);
  updateCA(2);
  updateCA(3);
  //printArrayDouble(CA, P*T, n);
  
  // load inputs
  double y0[P];
  double y1[P];
  double y2[P];
  double y3[P];
  double U[m*P];
  //readArrayFromFile("y0new.txt",y0);
  readArrayFromFile("y1.txt",y1);
  readArrayFromFile("y2.txt",y2);
  readArrayFromFile("y3.txt",y3);
  //readArrayFromFile("Uvector.txt",U);

  // Test YBu matrix
  printf("Debug Printing \n");
  printArrayDouble(y0, P, 1);
  printf("\n");
  printArrayDouble(y1, P, 1);
  printf("\n");
  printArrayDouble(y2, P, 1);
  printf("\n");
  printArrayDouble(y3, P, 1);
  printf("\n");

  //printf("\n");
  //printArrayDouble(y1, P, 1);
  //printf("\n");
  //printArrayDouble(y2, P, 1);
  printf("\n");
  //printArrayDouble(y3, P, 1);
  updateYBu(0,y0,U,YBu);
  //updateYBu(1,y1,U,YBu);
  printf("\n");
//  printArrayDouble(YBu,P*T,1);
  //updateYBu(2,y2,U,YBu);
  //updateYBu(3,y3,U,YBu);
  //printArrayDouble(YBu,P*T,1);

  return 0;
}
