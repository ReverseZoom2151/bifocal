#ifndef TEST_UTIL_H
#define TEST_UTIL_H

// Tiny hand-rolled assertion helpers. No external framework. A failing check
// records a failure and prints a diagnostic; main() returns non-zero if any
// check failed so the Makefile and CI can detect it.

#include <stdio.h>
#include <math.h>

static int g_checks_run = 0;
static int g_checks_failed = 0;

// Boolean expectation.
#define EXPECT(cond, msg)                                              \
  do {                                                                 \
    g_checks_run++;                                                    \
    if (!(cond)) {                                                     \
      g_checks_failed++;                                               \
      printf("  FAIL [%s:%d] %s\n", __func__, __LINE__, (msg));        \
    }                                                                  \
  } while (0)

// Floating point closeness expectation.
#define EXPECT_NEAR(actual, expected, eps, msg)                        \
  do {                                                                 \
    g_checks_run++;                                                    \
    double _a = (double)(actual);                                      \
    double _e = (double)(expected);                                    \
    if (fabs(_a - _e) > (eps)) {                                       \
      g_checks_failed++;                                               \
      printf("  FAIL [%s:%d] %s: expected %g, got %g (tol %g)\n",      \
             __func__, __LINE__, (msg), _e, _a, (double)(eps));        \
    }                                                                  \
  } while (0)

// Print a summary and return a process exit code (0 = all pass).
static int test_summary(const char* suite) {
  int passed = g_checks_run - g_checks_failed;
  printf("\n%s summary: %d checks, %d passed, %d failed\n",
         suite, g_checks_run, passed, g_checks_failed);
  if (g_checks_failed == 0) {
    printf("%s: ALL TESTS PASSED\n", suite);
    return 0;
  }
  printf("%s: TESTS FAILED\n", suite);
  return 1;
}

#endif
