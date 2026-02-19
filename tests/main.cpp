#include "test_framework.hpp"
#include <iostream>

namespace test {
std::vector<TestRegistration> &get_tests() {
  static std::vector<TestRegistration> tests;
  return tests;
}
} // namespace test

int main() {
  int passed = 0;
  int failed = 0;

  for (const auto &test : test::get_tests()) {
    try {
      test.func();
      std::cout << "[PASS] " << test.name << std::endl;
      passed++;
    } catch (const std::exception &e) {
      std::cout << "[FAIL] " << test.name << ": " << e.what() << std::endl;
      failed++;
    } catch (...) {
      std::cout << "[FAIL] " << test.name << ": Unknown error" << std::endl;
      failed++;
    }
  }

  std::cout << "Tests finished. Passed: " << passed << ", Failed: " << failed
            << std::endl;
  return failed > 0 ? 1 : 0;
}
