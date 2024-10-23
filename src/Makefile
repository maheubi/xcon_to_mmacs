CXX = g++
CXXFLAGS = -std=c++14 -Wall
LDFLAGS = -pthread
INCLUDES = -I../../include
LDPATH = -L../../lib
LIBS = -lZbCom

xcon_to_mmacs: xctomm.o
	$(CXX) $(LDFLAGS) $(LDPATH) -o -ld xcon_to_mmacs xctomm.o $(LIBS)

xctomm.o:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -ld xctomm.cpp

clean:
	rm -f *.o xcon_to_mmacs

xcon_to_mmacs: cmdvel_driver.o
	$(CXX) $(LDFLAGS) $(LDPATH) -o -ld xcon_to_mmacs cmdvel_driver.o $(LIBS)

xctomm.o:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -ld cmdvel_driver.cpp

clean:
	rm -f *.o xcon_to_mmacs
