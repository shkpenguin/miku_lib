#include "miku/lut.hpp"
#include "test_framework.hpp"

TEST_CASE(test_lut_get_value) {
  LookupTable lut({{0.0f, 0.0f}, {10.0f, 100.0f}});

  CHECK_CLOSE(lut.get_value(0.0f), 0.0f, 0.001);
  CHECK_CLOSE(lut.get_value(10.0f), 100.0f, 0.001);

  CHECK_CLOSE(lut.get_value(5.0f), 50.0f, 0.001);

  CHECK_CLOSE(lut.get_value(-5.0f), 0.0f, 0.001);
  CHECK_CLOSE(lut.get_value(15.0f), 100.0f, 0.001);
}
