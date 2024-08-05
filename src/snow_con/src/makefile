CC = g++
CCFLAGS =  -c -Wall -pthread  -O1
LDFLAGS =  -lrt -lm -lstdc++ -std=c++11 -pthread -lwiringPi

EXEC = main
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)


$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC) $(LDFLAGS)  

%.o:%.cpp
	$(CC) $(CCFLAGS) $< -o $@

clean: 
	rm -f $(EXEC) $(OBJECTS) 




