# Host-side unit tests for the Bifocal pure-logic classes.
#
# Usage:
#   make test     build and run every test, print a pass/fail summary
#   make build    build the test executables only
#   make clean     remove built executables
#
# The Arduino API is stubbed by arduino_stub.h so the firmware headers compile
# on a normal PC. No hardware and no external test framework required.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -I../firmware/line_following -I. -Wall

# .exe suffix so this works from a Windows shell as well as a POSIX one.
EXE := .exe

TESTS := test_kinematics test_variance
BINS  := $(addsuffix $(EXE),$(TESTS))

.PHONY: test build clean

build: $(BINS)

test_kinematics$(EXE): test_kinematics.cpp arduino_stub.h test_util.h \
                       ../firmware/line_following/kinematics.h
	$(CXX) $(CXXFLAGS) -o $@ test_kinematics.cpp

test_variance$(EXE): test_variance.cpp arduino_stub.h test_util.h \
                     ../firmware/line_following/analoglinesensors.h
	$(CXX) $(CXXFLAGS) -o $@ test_variance.cpp

# Build all tests, run each, and fail the whole target if any test fails.
test: build
	@fail=0; \
	for t in $(TESTS); do \
	  echo "=== running $$t ==="; \
	  ./$$t$(EXE) || fail=1; \
	  echo; \
	done; \
	if [ $$fail -eq 0 ]; then \
	  echo "ALL TEST SUITES PASSED"; \
	else \
	  echo "ONE OR MORE TEST SUITES FAILED"; \
	fi; \
	exit $$fail

clean:
	rm -f $(BINS)
