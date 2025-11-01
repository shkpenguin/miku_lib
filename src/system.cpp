#include <deque>
#include <memory>
#include "api.h"
#include "motions.h"
#include "mcl.h"

std::deque<std::shared_ptr<MotionPrimitive>> motion_queue;
pros::Mutex queue_mutex;
std::shared_ptr<MotionPrimitive> current_motion = nullptr;

void queue_motion(std::shared_ptr<MotionPrimitive> motion) {
    queue_mutex.take();
    motion_queue.push_back(motion);
    queue_mutex.give();
}

void queue_after_current(std::shared_ptr<MotionPrimitive> motion) {
    queue_mutex.take();
    if (current_motion == nullptr) {
        motion_queue.push_front(motion);
    } else {
        auto it = motion_queue.begin();
        ++it; // skip current motion
        motion_queue.insert(it, motion);
    }
    queue_mutex.give();
}

void system_task(void*) {
    uint32_t prev_time = pros::millis();

    while (true) {

        update_odom();
        update_particles();

        if (!current_motion) {
            queue_mutex.take();
            if (!motion_queue.empty()) {
                current_motion = motion_queue.front();
                motion_queue.pop_front();
                current_motion->start();
            }
            queue_mutex.give();
        }

        if (current_motion) {
            bool done = current_motion->is_done();

            // Evaluate events
            for (auto& e : current_motion->events) {
                if (!e.triggered && e.condition()) {
                    e.action();
                    e.triggered = true;
                }
            }

            if (done) {
                current_motion = nullptr; // move to next motion next loop
            } else {
                current_motion->update();
            }
        }

        pros::Task::delay_until(&prev_time, DELTA_TIME); // 10 ms loop
    }
}