# Bifocal offline replay / simulation harness.
# Host build only (no hardware). Reuses the firmware steering.h and the test
# Arduino stub via include paths.
#
#   make       build the harness
#   make run   build and run it (uses the default fixed seed)
#   make clean remove the built binary

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -O2
INCLUDES  = -I../firmware/line_following -I../tests

TARGET = replay
SRC    = replay.cpp

$(TARGET): $(SRC) ../firmware/line_following/steering.h ../tests/arduino_stub.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SRC)

.PHONY: run clean
run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
