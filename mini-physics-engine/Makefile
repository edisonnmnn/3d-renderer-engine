CXX      := c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Isrc
SDL_FLAGS := $(shell pkg-config --cflags --libs sdl2)

SRC := src/main.cpp
BIN := physics-engine

.PHONY: all run clean

all:
	$(CXX) $(CXXFLAGS) $(SRC) $(SDL_FLAGS) -o $(BIN)

run: all
	./$(BIN)

clean:
	rm -f $(BIN)
