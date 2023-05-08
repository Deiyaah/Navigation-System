CC = g++
CC_FLAGS = -Wall -g -std=c++11
OBJ = server.o dijkstra.o digraph.o
TARGET = server
PIPES = inpipe outpipe

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET)

server.o: server.cpp
	$(CC) $(CC_FLAGS) -c server.cpp

dijkstra.o: dijkstra.cpp
	$(CC) $(CC_FLAGS) -c dijkstra.cpp

digraph.o: digraph.cpp
	$(CC) $(CC_FLAGS) -c digraph.cpp

clean:
	rm -f $(OBJ) $(TARGET) $(PIPES) $(OBJ) *.out
	rm -f '../inpipe' '../outpipe'