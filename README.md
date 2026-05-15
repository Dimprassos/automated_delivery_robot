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
  an e-puck robot and a basic obstacle.
- `controllers/delivery_controller/` contains a Python controller split into:
  - `delivery_controller.py`: mission state machine.
  - `movement.py`: differential wheel commands.
  - `sensors.py`: e-puck proximity sensor helpers.
  - `odometry.py`: wheel encoder odometry with command-based fallback.
  - `navigation.py`: waypoint steering controller.
  - `mission_config.py`: named points and routes.
  - `metrics.py`: mission timing and obstacle avoidance counter.
  - `fsm.py`: robot mission states.

The default mission is `BASE -> POINT_A -> BASE`.

Controller arguments:

- `POINT_A`: run the default delivery route.
- `POINT_B`: run the shorter delivery route.
- `OBSTACLES`: enable proximity-sensor obstacle avoidance.

Obstacle avoidance is disabled by default so the baseline delivery mission can
be tested independently. Enable `OBSTACLES` only when running an obstacle
scenario.

The controller now runs a first point-to-point delivery mission. It estimates
pose with wheel encoder odometry when available, follows named waypoints,
temporarily overrides navigation for obstacle avoidance, records delivery, and
returns to base.

## Recommended Next Milestone

Improve the first minimum viable delivery mission:

1. Tune the waypoint coordinates and obstacle positions after running the world
   in Webots.
2. Add more delivery points and routes that match the final university layout.
3. Improve obstacle recovery so the robot returns to the route heading more
   smoothly.
4. Add repeated scenario runs to calculate success rate.
5. Extend localization with gyro or compass support if the simulated robot model
   provides it.

See `docs/implementation_plan.md` for a fuller roadmap.
