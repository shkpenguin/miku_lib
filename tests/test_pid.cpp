#include "miku/pid.hpp"
#include "test_framework.hpp"

TEST_CASE(test_pid_initialization) {
  PIDGains gains(1.0, 0.0, 0.0);
  PID pid(gains);

}

TEST_CASE(test_pid_proportional) {
  PIDGains gains(2.0, 0.0, 0.0);
  PID pid(gains);

  float output = pid.update(10.0);

  CHECK_CLOSE(output, 20.0, 0.001);
}

TEST_CASE(test_pid_integral) {
  PIDGains gains(0.0, 1.0, 0.0);
  PID pid(gains);

  float output = pid.update(10.0);

  CHECK_CLOSE(output, 0.0, 0.001);

  output = pid.update(20.0);

  CHECK_CLOSE(output, 5.0, 0.001);
}

TEST_CASE(test_pid_derivative) {
  PIDGains gains(0.0, 0.0, 0.5);
  PID pid(gains);

  float output = pid.update(10.0);

  CHECK_CLOSE(output, 5.0, 0.001);
}

TEST_CASE(test_pid_reset) {
  PIDGains gains(0.0, 1.0, 0.0);
  PID pid(gains);

  pid.update(10.0);
  pid.reset();

  float output = pid.update(10.0);

  CHECK_CLOSE(output, 0.0, 0.001);
}

TEST_CASE(test_pid_windup) {

  PIDGains gains(0.0, 1.0, 0.0);
  PID pid(gains, 5.0);

  float output = pid.update(10.0);
  CHECK_CLOSE(output, 0.0, 0.001);

  output = pid.update(4.0);
  CHECK_CLOSE(output, -3.0, 0.001);
}

TEST_CASE(test_pid_no_sign_flip) {
  PIDGains gains(0.0, 1.0, 0.0);

  PID pid(gains, 0.0, false);

  float output = pid.update(10.0);
  CHECK_CLOSE(output, 5.0, 0.001);

  output = pid.update(-10.0);
  CHECK_CLOSE(output, -5.0, 0.001);
}

TEST_CASE(test_pid_rectangular) {
  PIDGains gains(0.0, 1.0, 0.0);

  PID pid(gains, 0.0, true, false);

  float output = pid.update(10.0);

  CHECK_CLOSE(output, 0.0, 0.001);
}
