
all: monteCarloSpatialGrid

CC = g++

CFLAGS +=  $(shell pkg-config --cflags x11 xi xmu gl glu)
LDFLAGS += $(shell pkg-config --libs x11 xi xmu gl glu) 

# GLUT does not provide .pc file for pk-config
LDFLAGS += -lglut

LDFLAGS += -lm

# Debug
CFLAGS += -Wall -g

OBJ = glut_window.o main.o timer.o utils.o

monteCarloSpatialGrid: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o monteCarloSpatialGrid

glut_window.o: glut_window.cpp
	$(CC) $(CFLAGS) -c glut_window.cpp

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

timer.o: timer.cpp
	$(CC) $(CFLAGS) -c timer.cpp

utils.o: utils.cpp
	$(CC) $(CFLAGS) -c utils.cpp
  
clean:
	rm -fv $(OBJ) monteCarloSpatialGrid

