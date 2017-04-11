/////////////////////////////
// Matrix helper functions //
/////////////////////////////

#include "matrix-helper.h"
#include <stdio.h>

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
      //tmprowX[i] = X[(r*colsX)+i];
      tmprowX[i] = X[r + i*rowsX];
    }

    for (int c = 0; c < colsY; ++c) { // 4 times

      // grab the correct column of the second matrix
      for (int j = 0; j < rowsY; ++j) {
        //tmpcolY[j] = Y[c+(j*colsY)];
        tmpcolY[j] = Y[c*rowsY + j];
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
 * Add/subtract vectors x and y together (both vectors must of of length len)
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
  //for(int j = 0; j < cols; ++j){
  for(int i = 0; i < rows; ++i){
    printf("  ");
    // Loop across row
    //for(int i = 0; i < rows; ++i){
    for(int j = 0; j < cols; ++j){
      //printf("%lf ", array[i*cols + j]);
      printf("%lf ", array[i + j*rows]);
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
  //for(int j = 0; j < cols; ++j){
  for(int i = 0; i < rows; ++i){
    printf("  ");
    // Loop across row
    //for(int i = 0; i < rows; ++i){
    for(int j = 0; j < cols; ++j){
      //printf("%lf ", array[i*cols + j]);
      printf("%d ", array[i + j*rows]);
    }
    // Print enter
    printf("\n");
  }
  printf("\n");
}

///////////////////////////////
// Print arrays to text file //
///////////////////////////////
char* doubleArrayToString(double data[], int length){
    
    int i = 0;
    int sigFigs = 8; // How many sig figs to keep
    int bufSize = sigFigs+2;
    char numBuf[bufSize]; // buffer for each element in array, +2 b/c null char and decimal point
    
    // Room for each number, and commas, and space
    // +1 for zero terminator
    // MUST FREE LATER!!!!!
    char* outputString = malloc((bufSize + 3)*length+1);

    // Loop through, converting each elemetn to a string, and concatenating
    // to output string
    for(i = 0; i < length; ++i){
        snprintf(numBuf, bufSize, "%f", data[i]);
        strcat(outputString, numBuf);
        strcat(outputString,", ");
    }
    return outputString;
}


int writeFile(char* filename, double data [], int length){
    // Convert data to string
    char* dataString = doubleArrayToString(data,length);

    // Initialize File object
    // printf("Break 1");
    
    FILE *fp;
    // printf("Break 2");

    // Open file to write
    char * mode = "w"; // w: writing mode
                       // a: writing in appending mode 
                       //   (still makes new file if doesnt exist)
    fp = fopen(filename,mode );

    // Print data to file
    fprintf(fp, "%s", dataString);

    // Close file
    fclose(fp);

    printf("%s",dataString);

    return 0;
}

///////////////////////////////////
// Read in arrays from text file //
///////////////////////////////////
/*
 * Read in 1D array from file (w/ comma delimiter) 
 */
void readArrayFromFile(const char* file_name, double* array) {
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


