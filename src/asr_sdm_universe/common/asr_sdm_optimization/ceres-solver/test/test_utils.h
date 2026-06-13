#ifndef CERES_TEST_UTILS_H_
#define CERES_TEST_UTILS_H_

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

inline void ExpectTrue(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}

inline void ExpectNear(double value,
                       double expected,
                       double tolerance,
                       const std::string& message) {
  if (std::abs(value - expected) > tolerance) {
    std::cerr << "FAIL: " << message << " (got " << value << ", expected "
              << expected << ")" << std::endl;
    std::exit(1);
  }
}

#endif  // CERES_TEST_UTILS_H_
