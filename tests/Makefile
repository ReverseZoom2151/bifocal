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

TESTS := test_kinematics test_variance test_kalman test_recovery \
         test_disagreement test_policy
BINS  := $(addsuffix $(EXE),$(TESTS))

# Any test may include any firmware header or the shared test helpers, so depend
# on all of them and rebuild if a header changes.
HDRS := $(wildcard ../firmware/line_following/*.h) arduino_stub.h test_util.h

.PHONY: test build clean

build: $(BINS)

%$(EXE): %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $<

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
