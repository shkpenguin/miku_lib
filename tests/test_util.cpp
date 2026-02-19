#include "miku/util.hpp"
#include "test_framework.hpp"

TEST_CASE(test_clamp) {
  CHECK_CLOSE(clamp(10, 0, 5), 5.0, 0.001);
  CHECK_CLOSE(clamp(-10, 0, 5), 0.0, 0.001);
  CHECK_CLOSE(clamp(3, 0, 5), 3.0, 0.001);
}

TEST_CASE(test_sign) {
  CHECK_CLOSE(sign(10), 1.0, 0.001);
  CHECK_CLOSE(sign(-10), -1.0, 0.001);
  CHECK_CLOSE(sign(0), 0.0, 0.001);
}

TEST_CASE(test_slew) {

  CHECK_CLOSE(slew(10, 0, 2), 2.0, 0.001);
  CHECK_CLOSE(slew(-10, 0, 2), -2.0, 0.001);
  CHECK_CLOSE(slew(1, 0, 2), 1.0, 0.001);
}

TEST_CASE(test_ema) {

  CHECK_CLOSE(ema(10, 0, 0.5), 5.0, 0.001);
  CHECK_CLOSE(ema(10, 0, 0.1), 1.0, 0.001);
  CHECK_CLOSE(ema(10, 10, 0.5), 10.0, 0.001);
}

TEST_CASE(test_find_quadrant) {
  CHECK_CLOSE(find_quadrant(Pose(1, 1)), 1.0, 0.001);
  CHECK_CLOSE(find_quadrant(Pose(-1, 1)), 2.0, 0.001);
  CHECK_CLOSE(find_quadrant(Pose(-1, -1)), 3.0, 0.001);
  CHECK_CLOSE(find_quadrant(Pose(1, -1)), 4.0, 0.001);

  CHECK_CLOSE(find_quadrant(Pose(0, 0)), 1.0, 0.001);
  CHECK_CLOSE(find_quadrant(Pose(-1, 0)), 2.0, 0.001);
}

TEST_CASE(test_list) {
  List<int> list({1, 2, 3});

  CHECK_CLOSE(list.get_value(), 1.0, 0.001);

  CHECK_CLOSE(list.cycle_forward(), 2.0, 0.001);
  CHECK_CLOSE(list.cycle_forward(), 3.0, 0.001);
  CHECK_CLOSE(list.cycle_forward(), 1.0, 0.001);

  CHECK_CLOSE(list.cycle_reverse(), 3.0, 0.001);
  CHECK_CLOSE(list.cycle_reverse(), 2.0, 0.001);
}

TEST_CASE(test_miku_atan2) {

  standard_radians angle = miku::atan2(1.0, 1.0);
  CHECK_CLOSE(angle.value, M_PI / 4, 0.001);
}
