# Bifocal offline replay / simulation harness.
# Host build only (no hardware). Reuses the firmware steering.h and the test
# Arduino stub via include paths.
#
#   make        build the replay harness
#   make run    build and run it (uses the default fixed seed)
#   make sweep  build and run the parameter sweep (writes sweep_results.csv)
#   make clean  remove the built binaries and CSV

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -O2
INCLUDES  = -I../firmware/line_following -I../tests

HDRS = harness.h ../firmware/line_following/steering.h ../tests/arduino_stub.h

TARGET      = replay
SWEEP       = sweepsim
SWEEP_CSV   = sweep_results.csv

$(TARGET): replay.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) replay.cpp

$(SWEEP): sweep.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(SWEEP) sweep.cpp

.PHONY: run sweep clean
run: $(TARGET)
	./$(TARGET)

# Build and run the parameter sweep. Writes sim/sweep_results.csv.
sweep: $(SWEEP)
	./$(SWEEP)

clean:
	rm -f $(TARGET) $(TARGET).exe $(SWEEP) $(SWEEP).exe $(SWEEP_CSV)
