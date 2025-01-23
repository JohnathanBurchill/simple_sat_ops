# Makefile for ephem with static library    Updated: January 2025

SHELL = /bin/sh
CC    = gcc -Wall -O2 -march=native -D_FORTIFY_SOURCE=2

objects = sgp4sdp4.o sgp_in.o sgp_math.o sgp_obs.o sgp_time.o solar.o
library = libsgp4sdp4.a

# Default target
all: ephem $(library)

# Target for the executable
ephem: main.o $(library)
	$(CC) -lm -o ephem main.o $(library)

# Target for the static library
$(library): $(objects)
	ar rcs $@ $(objects)

# Object file dependencies
$(objects): sgp4sdp4.h

# Clean up object files and the executable
.PHONY: clean
clean:
	-rm -f *.o *~ ephem $(library)

# More thorough cleanup
.PHONY: distclean
distclean: clean
	-rm -f ephemrm -f $(PROJECT)
