# Produced by CVXGEN, 2017-03-26 23:36:38 -0400.
# CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
# The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
# CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
# applications without prior written permission from Jacob Mattingley.

# Filename: Makefile.
# Description: Basic Makefile.
OPT =  -std=c99 -Wall -Os
# libmath is needed for sqrt, which is used only for reporting the gap. Can
# remove if desired for production solvers..
LDLIBS = -lm
CFLAGS = $(OPT) $(INCLUDES)
CC = gcc
.PHONY: all
all: SSE_ang 
#testsolver: solver.o matrix_support.o ldl.o testsolver.o util.o CVX_inputs.o matrix-helper.o
SSE_ang: solver_ang.o matrix_support.o ldl.o SSE_ang.o util.o CVX_inputs_ang.o matrix-helper.o
# Include util.o for random functions and easy matrix printing.
#testsolver: solver.o matrix_support.o ldl.o util.o testsolver.o
CVX_inputs_ang.o: CVX_inputs_ang.c CVX_inputs_ang.h
matrix-helper.o: matrix-helper.c matrix-helper.h
solver_ang.o: solver_ang.h
matrix_support.o: solver_ang.h
ldl.o: solver_ang.h
util.o: solver_ang.h
#testsolver.o: solver.h CVX_inputs.h matrix-helper.h
SSE_ang.o: solver_ang.h CVX_inputs_ang.h matrix-helper.h
.PHONY : cleans
clean :
	del *.o testsolver
