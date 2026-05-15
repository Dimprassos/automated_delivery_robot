import sys

from controller import Robot

from fsm import RobotState
from metrics import MissionMetrics
from mission_config import (
    ARRIVAL_THRESHOLD,
    DEFAULT_DELIVERY_POINT,
    DELIVERY_ROUTES,
    INITIAL_POSE,
    MISSION_TIMEOUT_SECONDS,
    OBSTACLE_AVOIDANCE_ENABLED,
    RETURN_ROUTES,
    WAYPOINTS,
    planned_route_distance,
)
from movement import MovementController
from navigation import WaypointNavigator
from odometry import Odometry
from sensors import SensorController


print("Delivery controller is running")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

movement = MovementController(robot)
sensors = SensorController(robot, timestep)
odometry = Odometry(robot, timestep, INITIAL_POSE)
navigator = WaypointNavigator(movement, ARRIVAL_THRESHOLD)
metrics = MissionMetrics(robot)

delivery_point = DEFAULT_DELIVERY_POINT
obstacle_avoidance_enabled = OBSTACLE_AVOIDANCE_ENABLED

for raw_arg in sys.argv[1:]:
    arg = raw_arg.strip().upper()

    if not arg:
        continue

    if arg in DELIVERY_ROUTES:
        delivery_point = arg
    elif arg in ["OBSTACLES", "AVOID_OBSTACLES", "--OBSTACLES"]:
        obstacle_avoidance_enabled = True
    else:
        print(f"Unknown controller argument '{raw_arg}', ignoring it")

planned_distance = planned_route_distance(delivery_point)

state = RobotState.IDLE
previous_state = None

active_route = []
route_index = 0
mission_start_time = None
avoidance_start_time = None
avoidance_phase = 0
avoidance_direction = "right"
avoidance_cooldown_until = 0.0
failure_reason = None

metrics_printed = False
last_status_print = 0.0


def should_avoid_obstacle(pose, target, current_time):
    if not obstacle_avoidance_enabled:
        return False

    if current_time < avoidance_cooldown_until:
        return False

    _, heading_error = navigator.get_target_error(pose, target)

    return sensors.obstacle_detected_front() and abs(heading_error) < 0.75


while robot.step(timestep) != -1:
    current_time = robot.getTime()
    pose = odometry.update(movement.get_last_speeds())

    if (
        state not in [RobotState.IDLE, RobotState.FINISHED, RobotState.ERROR]
        and mission_start_time is not None
        and current_time - mission_start_time > MISSION_TIMEOUT_SECONDS
    ):
        failure_reason = "Mission timeout"
        movement.stop()
        state = RobotState.ERROR

    if state == RobotState.IDLE:
        print("State: IDLE")
        metrics.start_mission()
        mission_start_time = current_time

        active_route = list(DELIVERY_ROUTES[delivery_point])
        route_index = 0
        state = RobotState.GO_TO_DESTINATION
        print(f"State: GO_TO_DESTINATION -> {delivery_point}")
        print(f"Obstacle avoidance enabled: {obstacle_avoidance_enabled}")

    elif state == RobotState.GO_TO_DESTINATION:
        target_name = active_route[route_index]
        target = WAYPOINTS[target_name]

        if current_time - last_status_print > 2.0:
            print(
                "Navigating to "
                f"{target_name}: pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
            )
            last_status_print = current_time

        if should_avoid_obstacle(pose, target, current_time):
            print("Obstacle detected - switching to AVOID_OBSTACLE")
            metrics.count_obstacle_avoidance()
            movement.stop()

            previous_state = state
            avoidance_start_time = current_time
            avoidance_phase = 0
            avoidance_direction = "right" if sensors.obstacle_more_on_left() else "left"

            state = RobotState.AVOID_OBSTACLE

        elif navigator.drive_to(pose, target):
            print(f"Reached waypoint: {target_name}")
            metrics.record_waypoint(target_name)
            route_index += 1

            if route_index >= len(active_route):
                movement.stop()
                state = RobotState.DELIVER

    elif state == RobotState.AVOID_OBSTACLE:
        elapsed = current_time - avoidance_start_time

        if avoidance_phase == 0:
            if avoidance_direction == "right":
                movement.turn_right(speed=1.8)
            else:
                movement.turn_left(speed=1.8)

            if elapsed > 0.7 and sensors.path_clear_front():
                print("Path is clear - returning to navigation")
                movement.stop()
                avoidance_cooldown_until = current_time + 1.5
                state = previous_state or RobotState.GO_TO_DESTINATION

            elif elapsed > 2.0:
                print("Avoidance timeout - returning to navigation")
                movement.stop()
                avoidance_cooldown_until = current_time + 1.5
                state = previous_state or RobotState.GO_TO_DESTINATION

    elif state == RobotState.DELIVER:
        print("State: DELIVER")
        print(f"Package delivered at {delivery_point}")
        metrics.record_delivery(delivery_point)

        active_route = list(RETURN_ROUTES[delivery_point])
        route_index = 0
        state = RobotState.RETURN_TO_BASE
        print("State: RETURN_TO_BASE")

    elif state == RobotState.RETURN_TO_BASE:
        target_name = active_route[route_index]
        target = WAYPOINTS[target_name]

        if current_time - last_status_print > 2.0:
            print(
                "Returning to "
                f"{target_name}: pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
            )
            last_status_print = current_time

        if should_avoid_obstacle(pose, target, current_time):
            print("Obstacle detected during return - switching to AVOID_OBSTACLE")
            metrics.count_obstacle_avoidance()
            movement.stop()

            previous_state = state
            avoidance_start_time = current_time
            avoidance_phase = 0
            avoidance_direction = "right" if sensors.obstacle_more_on_left() else "left"

            state = RobotState.AVOID_OBSTACLE

        elif navigator.drive_to(pose, target):
            print(f"Reached waypoint: {target_name}")
            metrics.record_waypoint(target_name)
            route_index += 1

            if route_index >= len(active_route):
                movement.stop()
                state = RobotState.FINISHED

    elif state == RobotState.ERROR:
        movement.stop()

        if not metrics_printed:
            print("State: ERROR")
            metrics.finish_mission(
                success=False,
                path_length=odometry.path_length,
                planned_distance=planned_distance,
                failure_reason=failure_reason,
            )
            metrics_printed = True

    elif state == RobotState.FINISHED:
        movement.stop()

        if not metrics_printed:
            print("State: FINISHED")
            metrics.finish_mission(
                success=True,
                path_length=odometry.path_length,
                planned_distance=planned_distance,
            )
            metrics_printed = True
