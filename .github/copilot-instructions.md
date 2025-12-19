# AI Agent Coding Guide for 8889A-pushback (PROS V5)

## Big Picture
- Architecture: PROS V5 C++ project using a custom `miku` framework for chassis control, motion primitives, and localization.
- Control Loop: `autonomous()` and `opcontrol()` run tasks that update sensors (`Miku.update_position()`), manage an async motion queue, and drive motors; see [src/main.cpp](../src/main.cpp).
- Motion System: Motions subclass `MotionPrimitive` with `start/update/stop/is_done` and are queued via `queue_motion()`; events can be attached to motions; see [include/miku/motions.h](../include/miku/motions.h) and implementations in [src/motions/](../src/motions/).
- Routing: Autonomous routines encapsulated as functions that enqueue motions; optional precomputed Bezier paths are stored per route; see [include/routes.h](../include/routes.h) and [src/routes/](../src/routes/).
- Configuration: Robot devices, constants, PID gains, LUTs, and localization are centralized in [include/config.h](../include/config.h) and [src/config.cpp](../src/config.cpp).

## Build, Upload, and Run
- Build: `make` (default target `quick`) produces package binaries under [bin/](../bin/). With `USE_PACKAGE=1`, default output is [bin/hot.package.bin](../bin/hot.package.bin). Make rules/flags live in [Makefile](../Makefile) and [common.mk](../common.mk).
- Toolchain: ARM `arm-none-eabi-*` via PROS kernel 4.1.2; template info in [project.pros](../project.pros).
- Upload: Use PROS CLI from the project root (ensure PROS CLI is installed and Brain connected):
  ```powershell
  pros mu          # Upload firmware to V5 Brain
  pros terminal    # Open serial terminal (monitor logs)
  ```
- Notes: Section sizes are printed after link; `make all` cleans then rebuilds. `compile_commands.json` exists for IntelliSense.

## Conventions and Patterns
- Units:
  - Distances in inches; velocities in in/s; angles are either `standard_radians` (math frame) or `compass_degrees` (0° = +Y, increasing CW). Many motion APIs accept `compass_degrees` for heading.
  - Motor commands in millivolts: `Miku.move_voltage(left, right)` in [-12000, 12000].
- Motion Lifecycle:
  - Implement `start()` (init timers/PIDs), `update()` (compute outputs/stop criteria), `stop()` (ensure motors stop), and `is_done()` (return `done`). Use `Timer` from [include/miku/time.h](../include/miku/time.h) and exits from config.
  - Default gains: Param structs use -1 to mean “use global gains” from [src/config.cpp](../src/config.cpp) (`turn_gains`, `drive_gains`).
  - Safety: Call `Miku.stop()` when completing or aborting a motion.
- Events:
  - Attach conditional or sequential events to a motion (`add_conditional_event`, `add_sequential_event`). Example below.
- Queueing:
  - Use `queue_motion(motion)` or `queue_after_current(motion)`. Access to the queue is guarded in helpers via `queue_mutex`; avoid manual access.
- Routes:
  - Define a function queuing motions and an accompanying `std::vector<std::reference_wrapper<BezierPath>>` of paths, then add to the `routes` vector in `precalculate_paths()`; see [src/main.cpp](../src/main.cpp#L17-L41) and route examples in [src/routes/](../src/routes/).
- Localization:
  - Particle filter fuses distance sensors; see `ParticleFilter mcl` in [src/config.cpp](../src/config.cpp#L45-L61). Teleop disables it by default via `Miku.set_use_particle_filtering(false)`.
- UI/Feedback:
  - Use `master.display(row, supplier)` and `master.rumble(pattern)` for diagnostics; see display helpers in [src/main.cpp](../src/main.cpp#L43-L92).

## Key Files
- Build: [Makefile](../Makefile), [common.mk](../common.mk), [project.pros](../project.pros)
- Entry/Loops: [src/main.cpp](../src/main.cpp), [include/main.h](../include/main.h)
- Motions: [include/miku/motions.h](../include/miku/motions.h), [src/motions/](../src/motions/)
- Routes: [include/routes.h](../include/routes.h), [src/routes/](../src/routes/)
- Config/Devices: [include/config.h](../include/config.h), [src/config.cpp](../src/config.cpp)
- Paths: [include/miku/mp.h](../include/miku/mp.h)

## Examples
- Queue a motion with events (pattern):
  ```cpp
  auto m = new MovePoint({48, -20}, 1500, {.reverse = true, .drive_max_volt_pct = 60});
  m->add_conditional_event({
    [] { return Miku.get_pose().distance_to(Point(48, -24)) < 5.0; },
    [] { lock_piston.set_value(true); intake.set(12000, 12000); }
  });
  queue_motion(m);
  ```
- Simple route definition (pattern):
  ```cpp
  // src/routes/my_route.cpp
  #include "miku/miku-api.h"
  BezierPath my_path({ /* ControlPoint(x,y,vel) ... */ });
  std::vector<std::reference_wrapper<BezierPath>> my_paths = {std::ref(my_path)};
  void my_route() {
    queue_motion(new TurnHeading(180, 800));
    queue_motion(new MovePoint({24, -24}, 1500));
  }
  // Declare in include/routes.h and add to routes in src/main.cpp::precalculate_paths()
  ```

## Gotchas
- Always `Timer.set(timeout)` then `reset()` in `start()`.
- `min_volt_pct` applies to absolute values; clamp and sign carefully.
- In `MovePoint`, near-target logic clamps turn power and may early-complete if the drive-side flips; see [src/motions/move_point.cpp](../src/motions/move_point.cpp).
- When adding routes, ensure `paths[i].get().calculate_waypoints()` is valid before queueing (see [src/main.cpp](../src/main.cpp#L29-L41)).

## External Libraries
- PROS kernel 4.1.2, liblvgl 8.3.8 (headers under [include/liblvgl/](../include/liblvgl/)), `fmt` headers under [include/fmt/](../include/fmt/), GIF support in [include/gif-pros/](../include/gif-pros/).

If anything here is unclear or missing (e.g., your upload workflow or route selection process), tell me what to refine and I’ll update this guide quickly.
