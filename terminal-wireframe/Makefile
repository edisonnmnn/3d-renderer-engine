CXX      := c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Isrc

SRC := src/main.cpp
BIN := wireframe

.PHONY: all run clean

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(BIN)

run: all
	./$(BIN)

clean:
	rm -f $(BIN)
