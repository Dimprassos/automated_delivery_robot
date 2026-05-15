import sys

from controller import Robot

from fsm import RobotState
from metrics import MissionMetrics
from mission_config import (
    BASE_ARRIVAL_THRESHOLD,
    DEFAULT_DELIVERY_POINT,
    DELIVERY_ARRIVAL_THRESHOLD,
    DELIVERY_POINTS,
    INITIAL_POSE,
    MISSION_TIMEOUT_SECONDS,
    NAV_GRAPH_EDGES,
    OBSTACLE_AVOIDANCE_ENABLED,
    WAYPOINTS,
)
from movement import MovementController
from odometry import Odometry
from path_follower import RoutePathFollower
from planner import GraphPlanner
from sensors import SensorController


print("Delivery controller is running")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

movement = MovementController(robot)
sensors = SensorController(robot, timestep)
odometry = Odometry(robot, timestep, INITIAL_POSE)
planner = GraphPlanner(WAYPOINTS, NAV_GRAPH_EDGES)
metrics = MissionMetrics(robot)

delivery_point = DEFAULT_DELIVERY_POINT
obstacle_avoidance_enabled = OBSTACLE_AVOIDANCE_ENABLED

for raw_arg in sys.argv[1:]:
    arg = raw_arg.strip().upper()

    if not arg:
        continue

    if arg in DELIVERY_POINTS:
        delivery_point = arg
    elif arg in ["OBSTACLES", "AVOID_OBSTACLES", "--OBSTACLES"]:
        obstacle_avoidance_enabled = True
    else:
        print(f"Unknown controller argument '{raw_arg}', ignoring it")

route_to_delivery = planner.plan("BASE", delivery_point)
route_to_base = planner.plan(delivery_point, "BASE")
planned_distance = planner.path_distance(route_to_delivery) + planner.path_distance(route_to_base)

state = RobotState.IDLE
previous_state = None

active_follower = None
mission_start_time = None
avoidance_start_time = None
avoidance_phase = 0
avoidance_direction = "right"
avoidance_cooldown_until = 0.0
failure_reason = None

metrics_printed = False
last_status_print = 0.0
last_route_progress = 0.0
last_route_progress_time = 0.0
recovery_start_time = 0.0
recovery_previous_state = None
recovery_direction = "left"


def create_follower(route, final_threshold):
    return RoutePathFollower(
        movement=movement,
        route_names=route,
        waypoints=WAYPOINTS,
        final_threshold=final_threshold,
    )


def should_avoid_obstacle(current_time):
    if not obstacle_avoidance_enabled:
        return False

    if current_time < avoidance_cooldown_until:
        return False

    return sensors.obstacle_detected_front()


def reset_progress_watch(current_time):
    global last_route_progress, last_route_progress_time

    last_route_progress = 0.0
    last_route_progress_time = current_time


def route_is_stalled(result, current_time):
    global last_route_progress, last_route_progress_time

    if result["done"] or result["motion_mode"] == "align":
        last_route_progress = result["progress"]
        last_route_progress_time = current_time
        return False

    if result["progress"] > last_route_progress + 0.01:
        last_route_progress = result["progress"]
        last_route_progress_time = current_time
        return False

    return result["linear_speed"] > 0.0 and current_time - last_route_progress_time > 2.5


def start_recovery(from_state, current_time, result):
    global recovery_start_time, recovery_previous_state, recovery_direction, last_route_progress_time

    recovery_start_time = current_time
    recovery_previous_state = from_state
    recovery_direction = "right" if sensors.obstacle_more_on_left() else "left"
    last_route_progress_time = current_time
    movement.stop()

    print(
        "Route progress stalled - backing up and realigning "
        f"(progress={result['progress']:.2f}m, lateral_error={result['lateral_error']:.2f}m)"
    )

    return RobotState.RECOVER


def record_passed_nodes(result):
    for waypoint_name in result["passed_nodes"]:
        print(f"Passed route node: {waypoint_name}")
        metrics.record_waypoint(waypoint_name)


def format_turn_radius(turn_radius):
    if abs(turn_radius) == float("inf"):
        return "straight"

    if abs(turn_radius) < 1e-6:
        return "pivot"

    direction = "left" if turn_radius > 0.0 else "right"
    return f"{direction} {abs(turn_radius):.2f}m"


def print_tracking_status(prefix, result):
    lookahead_x, lookahead_y = result["lookahead_point"]
    eta = 0.0

    if result["linear_speed"] > 0.0:
        eta = result["remaining"] / result["linear_speed"]

    print(
        f"{prefix}: pose=({result['pose'][0]:.2f}, {result['pose'][1]:.2f}, {result['pose'][2]:.2f}), "
        f"remaining={result['remaining']:.2f}m, "
        f"lookahead={result['lookahead_distance']:.2f}m -> ({lookahead_x:.2f}, {lookahead_y:.2f}), "
        f"lateral_error={result['lateral_error']:.2f}m, "
        f"mode={result['motion_mode']}, "
        f"turn_radius={format_turn_radius(result['turn_radius'])}, "
        f"speed={result['linear_speed']:.2f}m/s, eta={eta:.1f}s"
    )


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

        active_follower = create_follower(route_to_delivery, DELIVERY_ARRIVAL_THRESHOLD)
        reset_progress_watch(current_time)
        state = RobotState.GO_TO_DESTINATION
        print(f"State: GO_TO_DESTINATION -> {delivery_point}")
        print(f"A* route to delivery: {' -> '.join(route_to_delivery)}")
        print("Tracking mode: pure pursuit lookahead")
        print(f"Obstacle avoidance enabled: {obstacle_avoidance_enabled}")

    elif state == RobotState.GO_TO_DESTINATION:
        result = active_follower.update(pose)
        record_passed_nodes(result)

        if current_time - last_status_print > 2.0:
            print_tracking_status(f"Tracking to {delivery_point}", result)
            last_status_print = current_time

        if result["done"]:
            movement.stop()
            print(f"Reached delivery point: {delivery_point}")
            metrics.record_waypoint(delivery_point)
            state = RobotState.DELIVER

        elif should_avoid_obstacle(current_time):
            print("Obstacle detected - switching to AVOID_OBSTACLE")
            metrics.count_obstacle_avoidance()
            movement.stop()

            previous_state = state
            avoidance_start_time = current_time
            avoidance_phase = 0
            avoidance_direction = "right" if sensors.obstacle_more_on_left() else "left"

            state = RobotState.AVOID_OBSTACLE

        elif route_is_stalled(result, current_time):
            metrics.count_obstacle_avoidance()
            state = start_recovery(state, current_time, result)

    elif state == RobotState.AVOID_OBSTACLE:
        elapsed = current_time - avoidance_start_time

        if avoidance_phase == 0:
            if avoidance_direction == "right":
                movement.turn_right(speed=1.8)
            else:
                movement.turn_left(speed=1.8)

            if elapsed > 0.7 and sensors.path_clear_front():
                print("Path is clear - returning to route tracking")
                movement.stop()
                avoidance_cooldown_until = current_time + 1.5
                state = previous_state or RobotState.GO_TO_DESTINATION

            elif elapsed > 2.0:
                print("Avoidance timeout - returning to route tracking")
                movement.stop()
                avoidance_cooldown_until = current_time + 1.5
                state = previous_state or RobotState.GO_TO_DESTINATION

    elif state == RobotState.RECOVER:
        elapsed = current_time - recovery_start_time

        if elapsed < 0.55:
            movement.move_backward(speed=1.8)
        elif elapsed < 1.15:
            if recovery_direction == "right":
                movement.turn_right(speed=1.7)
            else:
                movement.turn_left(speed=1.7)
        else:
            movement.stop()
            reset_progress_watch(current_time)
            print("Recovery complete - returning to route tracking")
            state = recovery_previous_state or RobotState.GO_TO_DESTINATION

    elif state == RobotState.DELIVER:
        print("State: DELIVER")
        print(f"Package delivered at {delivery_point}")
        metrics.record_delivery(delivery_point)

        active_follower = create_follower(route_to_base, BASE_ARRIVAL_THRESHOLD)
        reset_progress_watch(current_time)
        state = RobotState.RETURN_TO_BASE
        print("State: RETURN_TO_BASE")
        print(f"A* route to base: {' -> '.join(route_to_base)}")

    elif state == RobotState.RETURN_TO_BASE:
        result = active_follower.update(pose)
        record_passed_nodes(result)

        if current_time - last_status_print > 2.0:
            print_tracking_status("Tracking to BASE", result)
            last_status_print = current_time

        if result["done"]:
            movement.stop()
            print("Reached base")
            metrics.record_waypoint("BASE")
            state = RobotState.FINISHED

        elif should_avoid_obstacle(current_time):
            print("Obstacle detected during return - switching to AVOID_OBSTACLE")
            metrics.count_obstacle_avoidance()
            movement.stop()

            previous_state = state
            avoidance_start_time = current_time
            avoidance_phase = 0
            avoidance_direction = "right" if sensors.obstacle_more_on_left() else "left"

            state = RobotState.AVOID_OBSTACLE

        elif route_is_stalled(result, current_time):
            metrics.count_obstacle_avoidance()
            state = start_recovery(state, current_time, result)

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
