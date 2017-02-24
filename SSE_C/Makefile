CC = gcc
CFLAGS = -std=c99 -pedantic -Wall
OBJECTS = inputs.o

all: inputs

inputs.o: inputs.c 
	$(CC) $(CFLAGS) -c inputs.c

inputs: $(OBJECTS) 
	$(CC) $(OBJECTS) -o inputs

clean:
	rm -f *.o inputs

