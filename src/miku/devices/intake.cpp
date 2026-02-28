#include "miku/devices/intake.hpp"
#include "config.hpp"

namespace miku {

Intake::Intake(std::shared_ptr<Motor> top, std::shared_ptr<Motor> middle, std::shared_ptr<Motor> bottom)
    : top_motor(top), middle_motor(middle), bottom_motor(bottom) {}

void Intake::execute_command(std::shared_ptr<Motor>& motor, const MotorCommand& cmd) {
    if (cmd.type == IntakeCommandType::VOLTAGE) {
        motor->move_voltage(cmd.value);
    } else {
        motor->move_velocity(cmd.value);
    }
}

void Intake::move_normal() {
    execute_command(top_motor, top_command);
    execute_command(middle_motor, middle_command);
    execute_command(bottom_motor, bottom_command);
}

bool Intake::is_jammed() const {
    bool low_velocity = bottom_motor->get_filtered_velocity() < LOW_VELOCITY_THRESHOLD;
    bool commanded_forward = top_command.value > 0 || middle_command.value > 0 || bottom_command.value > 0;
    return low_velocity && commanded_forward;
}

// NOTE: unjam is now executed as a queued command (see queue_command).
// The old direct/unmanaged unjam routine has been removed — anti-jam will
// push an unjam QueueEntry into `command_queue` (higher priority) so that
// unjams behave like any other short, time-bound intake command.

// Monitor the jam condition and — when the persistence and cooldown
// criteria are met — push a short unjam entry into the intake queue. This
// makes anti-jam behave like any other queued intake command and avoids
// tightly coupling the unjam runtime to the intake update loop.
void Intake::handle_jam_detection(uint32_t now) {
    if (is_jammed()) {
        if (jam_detect_time == 0) {
            jam_detect_time = now;
        }

        if (now - jam_detect_time > JAM_THRESHOLD_MS) {
            // Only queue an unjam if cooldown elapsed and there isn't already
            // an identical unjam queued at the front.
            if (now - last_unjam_end_time > UNJAM_COOLDOWN_MS) {
                // push high-priority unjam so it runs immediately
                push_unjam_to_queue(true);
                // reset jam detect timer so we don't push repeatedly
                jam_detect_time = 0;
            } else {
                move_normal();
            }
        } else {
            move_normal();
        }
    } else {
        jam_detect_time = 0;
        move_normal();
    }
}

void Intake::set(const MotorCommand& top_cmd, const MotorCommand& middle_cmd, const MotorCommand& bottom_cmd) {
    top_command = top_cmd;
    middle_command = middle_cmd;
    bottom_command = bottom_cmd;
}

// Backwards compatible two-arg overload
void Intake::set(const MotorCommand& top_cmd, const MotorCommand& bottom_cmd) {
    set(top_cmd, MotorCommand(0.0f, IntakeCommandType::VOLTAGE), bottom_cmd);
}

void Intake::set(float top_voltage, float middle_voltage, float bottom_voltage) {
    top_command = MotorCommand(top_voltage, IntakeCommandType::VOLTAGE);
    middle_command = MotorCommand(middle_voltage, IntakeCommandType::VOLTAGE);
    bottom_command = MotorCommand(bottom_voltage, IntakeCommandType::VOLTAGE);
}

// Backwards compatible two-arg overload
void Intake::set(float top_voltage, float bottom_voltage) {
    set(top_voltage, bottom_voltage, bottom_voltage);
}

void Intake::set_velocity(float top_vel, float middle_vel, float bottom_vel) {
    top_command = MotorCommand(top_vel, IntakeCommandType::VELOCITY);
    middle_command = MotorCommand(middle_vel, IntakeCommandType::VELOCITY);
    bottom_command = MotorCommand(bottom_vel, IntakeCommandType::VELOCITY);
}

// Backwards compatible two-arg overload
void Intake::set_velocity(float top_vel, float bottom_vel) {
    set_velocity(top_vel, bottom_vel, bottom_vel);
}

void Intake::set_top(const MotorCommand& cmd) {
    top_command = cmd;
}

void Intake::set_top(float voltage) {
    top_command = MotorCommand(voltage, mV);
}

void Intake::set_top_velocity(float velocity) {
    top_command = MotorCommand(velocity, rpm);
}

void Intake::set_middle(const MotorCommand& cmd) {
    middle_command = cmd;
}

void Intake::set_middle(float voltage) {
    middle_command = MotorCommand(voltage, mV);
}

void Intake::set_middle_velocity(float velocity) {
    middle_command = MotorCommand(velocity, rpm);
}

void Intake::set_bottom(const MotorCommand& cmd) {
    bottom_command = cmd;
}

void Intake::set_bottom(float voltage) {
    bottom_command = MotorCommand(voltage, mV);
}

void Intake::set_bottom_velocity(float velocity) {
    bottom_command = MotorCommand(velocity, rpm);
} 

void Intake::load() {
    lock_piston.set_value(true);
    anti_jam_enabled = false;
    set(12000.0f, 12000.0f, 12000.0f);
}

void Intake::score() {
    lock_piston.set_value(false);
    // anti_jam_enabled = true;
    set(12000.0f, 12000.0f, 12000.0f);
}

void Intake::stop() {
    anti_jam_enabled = false;
    set(0.0f, 0.0f, 0.0f);
}

void Intake::set_anti_jam(bool enabled) {
    anti_jam_enabled = enabled;
    if (!enabled) {
        // clear any transient jam state
        jam_detect_time = 0;
    }
}

// Queue API implementations
void Intake::queue_command(const MotorCommand& top_cmd,
                            const MotorCommand& middle_cmd,
                            const MotorCommand& bottom_cmd,
                            uint32_t duration_ms,
                            bool front) {
    QueueEntry e;
    e.top = top_cmd;
    e.middle = middle_cmd;
    e.bottom = bottom_cmd;
    e.duration_ms = duration_ms;
    e.start_time = 0;
    // heuristically mark standard unjam entries (internal use)
    e.is_unjam = (duration_ms == UNJAM_DURATION_MS &&
                  top_cmd.type == IntakeCommandType::VOLTAGE &&
                  top_cmd.value == UNJAM_VOLTAGE &&
                  middle_cmd.type == IntakeCommandType::VOLTAGE &&
                  middle_cmd.value == UNJAM_VOLTAGE &&
                  bottom_cmd.type == IntakeCommandType::VOLTAGE &&
                  bottom_cmd.value == UNJAM_VOLTAGE);

    queue_mutex.take();
    if (front) command_queue.push_front(e);
    else command_queue.push_back(e);
    queue_mutex.give();
}

void Intake::queue_spin(float voltage, uint32_t duration_ms, bool front) {
    MotorCommand cmd(voltage, IntakeCommandType::VOLTAGE);
    queue_command(cmd, cmd, cmd, duration_ms, front);
}

void Intake::clear_queue() {
    queue_mutex.take();
    command_queue.clear();
    queue_mutex.give();
}

void Intake::cancel_current_command() {
    queue_mutex.take();
    if (!command_queue.empty()) command_queue.pop_front();
    queue_mutex.give();
}

size_t Intake::queued_size() const {
    // not strictly thread-safe but good enough for diagnostics
    return command_queue.size();
}

void Intake::push_unjam_to_queue(bool front) {
    MotorCommand u(UNJAM_VOLTAGE, IntakeCommandType::VOLTAGE);
    queue_command(u, u, u, UNJAM_DURATION_MS, front);
}

void Intake::update() {
    uint32_t now = pros::millis();

    // 1) If there are queued commands, execute the active entry (highest
    // priority = front of deque). Queued commands override default commands
    // until they complete or are cancelled.
    queue_mutex.take();
    if (!command_queue.empty()) {
        QueueEntry &active = command_queue.front();
        if (active.start_time == 0) active.start_time = now;

        // execute the queued command
        execute_command(top_motor, active.top);
        execute_command(middle_motor, active.middle);
        execute_command(bottom_motor, active.bottom);

        // check for completion (duration_ms == 0 => run until popped)
        if (active.duration_ms > 0 && (now - active.start_time >= active.duration_ms)) {
            bool was_unjam = active.is_unjam;
            command_queue.pop_front();
            queue_mutex.give();

            if (was_unjam) {
                // start unjam cooldown so we don't immediately re-enter
                last_unjam_end_time = now;
            }
            return; // we've already executed the queued command this tick
        }
        queue_mutex.give();
        return;
    }
    queue_mutex.give();

    // 2) No queued commands — fall back to anti-jam detection or normal
    // command behavior.
    if (!anti_jam_enabled) {
        move_normal();
        return;
    }

    // anti-jam flow may push an unjam entry into the queue; jam detection
    // will call move_normal() when no action is required
    handle_jam_detection(now);
}

} // namespace miku
