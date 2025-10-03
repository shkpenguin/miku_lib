#include "main.h"
#include "motor.h"
#include "util.h"

miku::MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports) {
    for(auto port : ports) {
        motors.push_back(new Motor(port));
    }
}

std::int32_t miku::MotorGroup::move(int voltage) {
    for(auto motor : motors) {
        motor->move(voltage);
    }
    return 1;
}

std::int32_t miku::MotorGroup::move_voltage(int voltage) {
    for(auto motor : motors) {
        motor->move_voltage(voltage);
    }
    return 1;
}

std::vector<double> miku::MotorGroup::get_estimated_velocities() {
    std::vector<double> velocities;
    for(auto motor : motors) {
        velocities.push_back(motor->get_estimated_velocity());
    }
    return velocities;
}
// Uses get_estimated_velocity() to return the average velocity of the group
double miku::MotorGroup::get_average_velocity() {
    auto velocities = get_estimated_velocities();
    double sum = std::accumulate(velocities.begin(), velocities.end(), 0.0);
    return sum / velocities.size();
}

double miku::MotorGroup::get_average_position() {
    double sum = 0;
    for(auto motor : motors) {
        sum += motor->get_position();
    }
    return sum / motors.size();
}

std::int32_t miku::MotorGroup::tare_position_all(void) const {
    for(auto motor : motors) {
        motor->tare_position();
    }
    return 1;
}

std::int32_t miku::MotorGroup::set_brake_mode_all(pros::motor_brake_mode_e mode) {
    for(auto motor : motors) {
        motor->set_brake_mode(mode);
    }
    return 1;
}

double miku::Motor::get_estimated_velocity() {

    u_int32_t current_time;
    double current_ticks = this->get_raw_position(&current_time);

    double delta_ticks = current_ticks - prev_ticks;
    double delta_time = 5 * std::round((current_time - last_time) / 5.0); 

    double prev_raw_velocity = prev_raw_velocities.back();
    double prev_filtered_velocity = prev_filtered_velocities.back();

    if(delta_time <= 0) {
        return prev_raw_velocity;
    }

    double raw_velocity = (delta_ticks / ticks_per_rev) / (delta_time / 60000.0);
    if(std::fabs(raw_velocity) > 1000) {
        return prev_raw_velocity;
    }

    prev_raw_velocities.push_back(raw_velocity);
    if(prev_raw_velocities.size() > std::max(sma_filter_size, median_filter_size)) {
        prev_raw_velocities.erase(prev_raw_velocities.begin());
    }

    double sum = std::accumulate(prev_raw_velocities.begin(), prev_raw_velocities.end(), 0.0);
    double sma_velocity = sum / prev_raw_velocities.size();

    prev_filtered_velocities.push_back(sma_velocity);
    if(prev_filtered_velocities.size() > median_filter_size) {
        prev_filtered_velocities.erase(prev_filtered_velocities.begin());
    }

    double median_velocity = 0;
    if(prev_filtered_velocities.size() & 1) {
        median_velocity = prev_filtered_velocities[prev_filtered_velocities.size() / 2];
    } else {
        median_velocity = (prev_filtered_velocities[prev_filtered_velocities.size() / 2 - 1] + prev_filtered_velocities[prev_filtered_velocities.size() / 2]) / 2.0;
    }

    double accel = (sma_velocity - prev_filtered_velocities.back()) / (delta_time / 1000.0);
    prev_accels.push_back(accel);
    if(prev_accels.size() > accel_filter_size) {
        prev_accels.erase(prev_accels.begin());
    }
    double max_accel = *std::max_element(prev_accels.begin(), prev_accels.end());

    double kA = 0.75 * (1 - (1 / (max_accel * (max_accel / 50) + 1.013)));
    double ema_velocity = ema(median_velocity, prev_estimated_velocity, kA);

    prev_estimated_velocity = ema_velocity;
    prev_ticks = current_ticks;
    last_time = current_time;

    return ema_velocity;

}