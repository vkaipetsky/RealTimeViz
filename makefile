//INCLUDE = -I/usr/include/
//LIBDIR  = -L/usr/X11R6/lib 
INCLUDE = -I/usr/include/ -I/opt/X11/include/
LIBDIR  = -L/opt/X11/lib/

COMPILERFLAGS = -Wall
CC = g++
CFLAGS = $(COMPILERFLAGS) $(INCLUDE)
LIBRARIES = -lX11 -lXi -lXmu -lglut -lGL -lGLU -lm 

monteCarloSpatialGrid : monteCarloSpatialGrid.cpp
	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBRARIES)  

