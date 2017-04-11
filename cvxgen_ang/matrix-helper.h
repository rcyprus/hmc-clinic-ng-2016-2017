// matrix-helper.h
// Contains matrix helper functions for CVX input functions

#ifndef MATRIX_HELPER_H_INCLUDED
#define MATRIX_HELPER_H_INCLUDED 1

/* 
 * Multiplies together matrices A and B (sizes must be compatible)
 */
void multiply(double* X, int rowsX, int colsX,
              double* Y, int rowsY, int colsY,
              double* XY);

/* 
 * Takes the dot product of vectors x and y (vectors must have length len)
 */
double dot(double* x, double* y, int len);

/* 
 * Adds vectors x and y together (both vectors must of of length len)
 */
void add(double* x, double* y, int len, double* xplusy);

void sub(double* x, double* y, int len, double* xminy); 

/*
 * Debugging print function (only 2D arrays)
 */
void printArrayDouble(double* array, int rows, int cols);

/*
 * Debugging print function (only 2D int arrays)
 */
void printArrayInt(int* array, int rows, int cols);

/*
 * Write a DOUBLE array to a file
 */
int writeFile(char* filename, double* data, int length);

/*
 * Convert a flat double array to a string
 */
char* doubleArrayToString(double* data, int length);

/*
 * Read in 1D array from file (w/ comma delimiter) 
 */
void readArrayFromFile(const char* file_name, double* array);

#endif


