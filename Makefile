
all: monteCarloSpatialGrid

CC = g++

CFLAGS +=  $(shell pkg-config --cflags x11 xi xmu gl glu)
LDFLAGS += $(shell pkg-config --libs x11 xi xmu gl glu) 

# GLUT does not provide .pc file for pk-config
LDFLAGS += -lglut

LDFLAGS += -lm

# Debug
CFLAGS += -Wall -g

OBJ = monteCarloSpatialGrid.o

monteCarloSpatialGrid: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o monteCarloSpatialGrid

monteCarloSpatialGrid.o: monteCarloSpatialGrid.cpp
	$(CC) $(CFLAGS) -c monteCarloSpatialGrid.cpp

clean:
	rm -fv $(OBJ) monteCarloSpatialGrid

