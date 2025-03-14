# Compiler
CXX = g++
CC = gcc

# Compiler flags
CXXFLAGS = -std=c++20 -Wall -Wextra -O2
LDFLAGS = -lpigpio -lrt

# Source files
SRC = main.cpp
OBJ = $(SRC:.cpp=.o)

# Output binary
TARGET = napa

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compilation
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJ) $(TARGET)

# Install pigpio (if not already installed)
install_pigpio:
	sudo apt-get install pigpio

.PHONY: all clean
