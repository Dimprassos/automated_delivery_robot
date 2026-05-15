# Implementation Plan

## Project Target

Develop a Webots simulation of an autonomous indoor delivery robot for a
university building. The robot should move between predefined points, avoid
obstacles, and report measurable mission results.

## What The Brief Requires

The report and presentation define the project around five capabilities:

- A university-like indoor map with corridors, corners, rooms, and obstacles.
- Predefined mission points such as base, offices, classrooms, or labs.
- Autonomous navigation from a start point to a destination.
- Obstacle detection and avoidance during movement.
- Evaluation using mission success, completion time, and navigation metrics.

The literature review points toward a mature architecture with perception,
localization, navigation, and evaluation. For this Webots project, the practical
version should be scoped so it can be finished reliably inside the simulator.

## Current Gap

The repository now has the first minimum viable mission: named waypoints,
odometry-based pose estimation, waypoint steering, mission return to base, and
metrics. Obstacle avoidance exists as an optional test mode, but is disabled by
default so the baseline delivery scenario can be evaluated independently. The
remaining gaps are:

- The current floor layout is still small and should be tuned in Webots.
- The obstacle recovery behavior is simple and should be tuned only after the
  baseline route is reliable.
- There is no full global path planner yet.
- The simulation does not yet run repeated trials for success-rate statistics.
- Gyro or compass fusion is not implemented yet.

## Target Architecture

Use a layered controller:

- Mission layer: decides the active goal and mission state.
- Localization layer: estimates robot pose from wheel encoders and optional gyro
  or compass data.
- Navigation layer: converts the current goal into steering commands.
- Obstacle layer: reacts to proximity sensors and temporarily overrides normal
  navigation.
- Metrics layer: records time, success, avoidances, path length, and failures.

This keeps the implementation close to the report's perception, localization,
navigation, and evaluation structure without requiring a full ROS2/Nav2 stack.

## Milestones

### Milestone 1: Realistic Test World

Definition of done:

- The world has a small indoor floor plan with corridors and rooms.
- There is one base point and at least two delivery points.
- Obstacles exist in positions that force the robot to react.
- Delivery points are visually marked and named in the world file.

Suggested map:

- `BASE`: robot start area.
- `POINT_A`: classroom or lab delivery point.
- `POINT_B`: office delivery point.
- Static obstacles: box in corridor, partial blockage near a corner.

### Milestone 2: Point-To-Point Navigation

Definition of done:

- The robot estimates its pose.
- The controller stores target coordinates for `BASE`, `POINT_A`, and
  `POINT_B`.
- The robot drives toward a selected target instead of relying on a timer.
- Arrival is detected using a distance threshold.

Implementation notes:

- Start with wheel encoder odometry if the e-puck wheel sensors are available.
- If encoder support is not available in the current Webots model, use a simple
  simulated localization helper for the first demo and document the limitation.
- Keep the first path simple: straight corridor plus one turn.

### Milestone 3: Obstacle Avoidance

Definition of done:

- The robot detects frontal obstacles with proximity sensors.
- The robot chooses an avoidance direction based on left/right sensor values.
- After avoidance, the robot resumes navigation toward the same target.
- The controller logs each avoidance event.

Suggested approach:

- Keep the current phased avoidance as the baseline.
- Improve it by returning to goal heading after the obstacle is cleared.
- Add a timeout and failure state for blocked paths.

### Milestone 4: Mission Metrics

Definition of done:

- Each run reports:
  - success or failure,
  - mission time,
  - obstacle avoidance count,
  - estimated path length,
  - target reached,
  - failure reason when applicable.
- Results can be copied into the final report/presentation.

Optional metrics:

- Success rate over multiple runs.
- Route efficiency: straight-line distance divided by estimated traveled path.
- Localization error, if ground truth is accessible in simulation.

### Milestone 5: Multi-Point Delivery

Definition of done:

- The robot can select among multiple delivery points.
- A simple route planner chooses an order or path between points.
- The project demonstrates at least two mission scenarios:
  - `BASE -> POINT_A -> BASE`
  - `BASE -> POINT_B -> BASE`

Advanced extension:

- Add a graph of corridors and use A* for global routing.
- Compare routes by estimated travel time, connecting the implementation to the
  Kim and Jung routing discussion from the literature review.

## Immediate Coding Task

The best next code change is to tune the mission after a Webots run. Use the
current `BASE -> POINT_A -> BASE` mission as the main demo, then adjust waypoint
coordinates, obstacle positions, sensor thresholds, and avoidance timings based
on observed behavior.

## Suggested Demo Scenarios

Use these for evaluation:

- Scenario 1: clear route from base to delivery point.
- Scenario 2: one obstacle in the corridor.
- Scenario 3: obstacle near a turn.
- Scenario 4: partially blocked corridor that forces avoidance timeout or a
  longer recovery.

For each scenario, record mission time, success, and obstacle avoidance count.
