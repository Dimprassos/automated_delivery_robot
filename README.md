# Autonomous Indoor Delivery Robot

Webots project for an autonomous mobile robot that delivers small items inside a
university building. The intended scenario is a robot moving between predefined
points such as offices, classrooms, and labs while avoiding obstacles and
recording mission performance.

## Source Brief

The project direction is based on:

- `Robotics report.pdf`
- `Robotics Presentation.pptx`

Core goals from the brief:

- Simulate a university-like indoor environment in Webots.
- Navigate autonomously between predefined delivery points.
- Detect and avoid obstacles using onboard sensors.
- Evaluate missions using success, completion time, and navigation efficiency.

## Current Implementation

- `worlds/automated_delivery_robot.wbt` contains the current Webots world with
  an e-puck robot in a redesigned university academic wing. The layout uses a
  main corridor, cross-corridor/atrium, classroom, teaching lab, research lab,
  faculty office, prep/storage room, study area, and mail/base room.
- `controllers/delivery_controller/` contains a Python controller split into:
  - `delivery_controller.py`: mission state machine.
  - `movement.py`: differential wheel commands.
  - `sensors.py`: e-puck proximity sensor helpers.
  - `odometry.py`: wheel encoder odometry with command-based fallback.
  - `navigation.py`: waypoint steering controller.
  - `mission_config.py`: named points and routes.
  - `metrics.py`: mission timing and obstacle avoidance counter.
  - `fsm.py`: robot mission states.

By default, the controller starts with a `BASE -> POINT_A` delivery job and
injects a demo `BASE -> POINT_B` job after 6 seconds. Jobs have a fixed pickup
point and fixed dropoff point. If a job needs an item from base, its pickup is
`BASE`; if a job is between rooms, its pickup can be `POINT_A` or `POINT_B`.
The first log lines print the initial job and every scheduled job. Use
`NO_JOBS` to disable the built-in demo job.

Current service locations:

- `BASE`: mail/loading room B01.
- `POINT_A`: teaching lab 112.
- `POINT_B`: faculty office 108.

Controller arguments:

- `POINT_A`: run the default `BASE -> POINT_A` initial job.
- `POINT_B`: run `BASE -> POINT_B` as the initial job.
- `JOB=BASE,POINT_B,6,1`: inject a new job while the robot is running. Format
  is `JOB=<PICKUP>,<DROPOFF>,<SECONDS>,<PRIORITY>`.
- `REQUEST=POINT_B,6,1`: shorthand for `JOB=BASE,POINT_B,6,1`.
- `JOB=POINT_A,POINT_B,10,1`: request a room-to-room delivery.
- `URGENT=POINT_A,POINT_B,10`: same as `JOB`, but defaults to priority `3`.
- `NO_JOBS`: disable the built-in demo job.
- `OBSTACLES`: enable proximity-sensor obstacle avoidance.

Obstacle avoidance is disabled by default so the baseline delivery mission can
be tested independently. Enable `OBSTACLES` only when running an obstacle
scenario.

The controller runs fixed-location pickup/dropoff jobs. It estimates pose with
wheel encoder odometry when available, follows named waypoints, queues scripted
jobs that arrive mid-route, temporarily overrides navigation for obstacle
avoidance, records completed jobs, and returns to base when no pending job
remains.

## Recommended Next Milestone

Improve the first minimum viable delivery mission:

1. Tune the waypoint coordinates and obstacle positions after running the world
   in Webots.
2. Add more delivery points and routes if the final demo needs more rooms than
   the current base, teaching lab, and faculty office.
3. Improve obstacle recovery so the robot returns to the route heading more
   smoothly.
4. Add repeated scenario runs to calculate success rate.
5. Extend localization with gyro or compass support if the simulated robot model
   provides it.

See `docs/implementation_plan.md` for a fuller roadmap.
