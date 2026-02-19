#include "miku/geometry.hpp"
#include "test_framework.hpp"

TEST_CASE(test_angle_conversion) {
  standard_degrees d(180.0);
  standard_radians r = d;
  CHECK_CLOSE(r.value, M_PI, 0.001);

  standard_radians r2(M_PI);
  standard_degrees d2 = r2;
  CHECK_CLOSE(d2.value, 180.0, 0.001);
}

TEST_CASE(test_angle_normalization) {
  standard_degrees d(370.0);
  CHECK_CLOSE(d.norm(), 10.0, 0.001);

  standard_degrees d2(-10.0);
  CHECK_CLOSE(d2.norm(), 350.0, 0.001);
}

TEST_CASE(test_angle_add) {
  standard_degrees d1(10.0);
  standard_degrees d2(20.0);
  standard_degrees sum = d1 + d2;
  CHECK_CLOSE(sum.value, 30.0, 0.001);
}

TEST_CASE(test_point_distance) {
  Point p1(0, 0);
  Point p2(3, 4);
  CHECK_CLOSE(p1.distance_to(p2), 5.0, 0.001);
}

TEST_CASE(test_pose_to_string) {
  Pose p(10.5, 20.1, M_PI / 2);

  CHECK_CLOSE(p.theta.compass().value, 0.0, 0.001);

}

TEST_CASE(test_geometry_operators) {
  standard_degrees d1(10.0);
  standard_degrees d2(20.0);

  CHECK_CLOSE(d2.operator-(d1).value, 10.0, 0.001);

  CHECK_CLOSE(d1.operator*(2).value, 20.0, 0.001);

  CHECK_CLOSE(d2.operator/(2).value, 10.0, 0.001);

  d1 += 5.0;
  CHECK_CLOSE(d1.value, 15.0, 0.001);
}

TEST_CASE(test_geometry_conversions) {

  standard_degrees std_deg(0.0);
  CHECK_CLOSE(std_deg.compass().value, 90.0, 0.001);

  standard_degrees std_north(90.0);
  CHECK_CLOSE(std_north.compass().value, 0.0, 0.001);
}
