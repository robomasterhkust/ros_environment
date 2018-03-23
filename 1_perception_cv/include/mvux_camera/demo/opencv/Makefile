CXX ?= g++
CFLAGS =`pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`  #-L../../lib/
#SHARELIBDIR= -L../../../lib/gcc-4.4.3/
INCLUDE= -I../../

all: main
main:  
	$(CXX) $(CFLAGS)  -o main  main.cpp $(LIBS)   $(INCLUDE)   -lMVSDK
clean:
	rm -f *.o 
	rm -f main 
