# Produced by CVXGEN, 2017-03-26 23:36:38 -0400.
# CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
# The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
# CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
# applications without prior written permission from Jacob Mattingley.

# Filename: Makefile.
# Description: Basic Makefile.
OPT = -Wall -Os
# libmath is needed for sqrt, which is used only for reporting the gap. Can
# remove if desired for production solvers..
LDLIBS = -lm
CFLAGS = $(OPT) $(INCLUDES)
CC = gcc
.PHONY: all
all: testsolver SSE_pos_offboard
testsolver: solver.o matrix_support.o ldl.o testsolver.o util.o CVX_inputs.o matrix-helper.o
SSE_pos_offboard: solver.o matrix_support.o ldl.o SSE_pos_offboard.o util.o CVX_inputs.o matrix-helper.o
# Include util.o for random functions and easy matrix printing.
#testsolver: solver.o matrix_support.o ldl.o util.o testsolver.o
CVX_inputs.o: CVX_inputs.c CVX_inputs.h
matrix-helper.o: matrix-helper.c matrix-helper.h
solver.o: solver.h
matrix_support.o: solver.h
ldl.o: solver.h
util.o: solver.h
testsolver.o: solver.h CVX_inputs.h matrix-helper.h
SSE_pos_offboard.o: solver.h CVX_inputs.h matrix-helper.h
.PHONY : clean
clean :
	del *.o testsolver
