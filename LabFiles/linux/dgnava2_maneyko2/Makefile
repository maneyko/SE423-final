CXX = arm-none-linux-gnueabi-gcc

targets = dgnava2_maneyko2

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
omapl138_gpiofuncs.o: omapl138_gpiofuncs.c 
	$(CXX) -c omapl138_gpiofuncs.c

dgnava2_maneyko2.o: dgnava2_maneyko2.c
	$(CXX) -c dgnava2_maneyko2.c	 

dgnava2_maneyko2: netapi.o omapl138_gpiofuncs.o dgnava2_maneyko2.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 
clean:
	rm -rf *.o $(targets)
  
