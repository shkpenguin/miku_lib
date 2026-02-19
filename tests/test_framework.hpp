#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace test {

struct TestRegistration {
  std::string name;
  std::function<void()> func;
};

std::vector<TestRegistration> &get_tests();

struct RegisterTest {
  RegisterTest(const std::string &name, std::function<void()> func) {
    get_tests().push_back({name, func});
  }
};

#define TEST_CASE(name)                                                        \
  void name();                                                                 \
  static test::RegisterTest register_##name(#name, name);                      \
  void name()

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "FAIL: " << #condition << " at " << __FILE__ << ":"         \
                << __LINE__ << std::endl;                                      \
      throw std::runtime_error("Test failed");                                 \
    }                                                                          \
  } while (0)

#define CHECK_CLOSE(a, b, epsilon)                                             \
  do {                                                                         \
    if (std::abs((a) - (b)) > (epsilon)) {                                     \
      std::cerr << "FAIL: " << #a << " (" << (a) << ") != " << #b << " ("      \
                << (b) << ") within " << (epsilon) << " at " << __FILE__       \
                << ":" << __LINE__ << std::endl;                               \
      throw std::runtime_error("Test failed");                                 \
    }                                                                          \
  } while (0)
}
