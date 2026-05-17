import math
import sys

from controller import Robot

from fsm import RobotState
from metrics import MissionMetrics
from mission_config import (
    BASE_ARRIVAL_THRESHOLD,
    DEFAULT_DELIVERY_DROPOFF,
    DEFAULT_DELIVERY_PICKUP,
    DEMO_JOB_DROPOFF,
    DEMO_JOB_PICKUP,
    DEMO_JOB_PRIORITY,
    DEMO_JOB_TIME_SECONDS,
    DEMO_JOBS_ENABLED,
    DELIVERY_ARRIVAL_THRESHOLD,
    DELIVERY_POINTS,
    INITIAL_POSE,
    MISSION_TIMEOUT_SECONDS,
    NAV_GRAPH_EDGES,
    OBSTACLE_AVOIDANCE_ENABLED,
    SERVICE_POINTS,
    WAYPOINTS,
)
from mission_manager import JobMissionManager, ScheduledDeliveryJob
from movement import MovementController
from odometry import Odometry
from path_follower import RoutePathFollower
from planner import GraphPlanner
from sensors import SensorController


SCRIPTED_JOB_DEFAULT_TIME = 6.0
BASE_NODE = "BASE"

LEG_PICKUP = "PICKUP"
LEG_DROPOFF = "DROPOFF"
LEG_RETURN_BASE = "RETURN_BASE"

print("Delivery controller is running")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

movement = MovementController(robot)
sensors = SensorController(robot, timestep)
odometry = Odometry(robot, timestep, INITIAL_POSE)
planner = GraphPlanner(WAYPOINTS, NAV_GRAPH_EDGES)
metrics = MissionMetrics(robot)

initial_pickup = DEFAULT_DELIVERY_PICKUP
initial_dropoff = DEFAULT_DELIVERY_DROPOFF
obstacle_avoidance_enabled = OBSTACLE_AVOIDANCE_ENABLED
scheduled_jobs = []
jobs_disabled = False


def is_number(value):
    try:
        float(value)
        return True
    except ValueError:
        return False


def parse_scheduled_job(spec, default_priority=0):
    normalized = spec.strip().upper()
    normalized = normalized.replace("->", ",").replace("@", ",")
    parts = [part.strip() for part in normalized.split(",") if part.strip()]

    if not parts:
        print("Empty delivery job argument, ignoring it")
        return None

    if len(parts) >= 2 and parts[0] in SERVICE_POINTS and parts[1] in SERVICE_POINTS:
        pickup = parts[0]
        dropoff = parts[1]
        rest = parts[2:]
    elif parts[0] in DELIVERY_POINTS:
        pickup = BASE_NODE
        dropoff = parts[0]
        rest = parts[1:]
    else:
        print(f"Unknown delivery job '{spec}', ignoring it")
        return None

    if pickup == dropoff:
        print(f"Invalid delivery job '{pickup}->{dropoff}', ignoring it")
        return None

    trigger_time = SCRIPTED_JOB_DEFAULT_TIME
    priority = default_priority

    if len(rest) >= 1:
        if is_number(rest[0]):
            trigger_time = max(0.0, float(rest[0]))
        else:
            print(f"Invalid job time '{rest[0]}', using {trigger_time:.1f}s")

    if len(rest) >= 2:
        try:
            priority = max(0, int(rest[1]))
        except ValueError:
            print(f"Invalid job priority '{rest[1]}', using {priority}")

    return ScheduledDeliveryJob(
        pickup=pickup,
        dropoff=dropoff,
        trigger_time=trigger_time,
        priority=priority,
    )


for raw_arg in sys.argv[1:]:
    clean_arg = raw_arg.strip()
    arg = clean_arg.upper()

    if not arg:
        continue

    if arg in DELIVERY_POINTS:
        initial_pickup = BASE_NODE
        initial_dropoff = arg
    elif arg in ["NO_REQUESTS", "--NO_REQUESTS", "NO_JOBS", "--NO_JOBS"]:
        jobs_disabled = True
    elif arg.startswith("REQUEST=") or arg.startswith("--REQUEST="):
        job = parse_scheduled_job(clean_arg.split("=", 1)[1])

        if job is not None:
            scheduled_jobs.append(job)
    elif arg.startswith("JOB=") or arg.startswith("--JOB="):
        job = parse_scheduled_job(clean_arg.split("=", 1)[1])

        if job is not None:
            scheduled_jobs.append(job)
    elif arg.startswith("URGENT=") or arg.startswith("--URGENT="):
        job = parse_scheduled_job(clean_arg.split("=", 1)[1], default_priority=3)

        if job is not None:
            scheduled_jobs.append(job)
    elif arg in ["OBSTACLES", "AVOID_OBSTACLES", "--OBSTACLES"]:
        obstacle_avoidance_enabled = True
    else:
        print(f"Unknown controller argument '{raw_arg}', ignoring it")

if jobs_disabled:
    scheduled_jobs = []
elif not scheduled_jobs and DEMO_JOBS_ENABLED:
    scheduled_jobs.append(
        ScheduledDeliveryJob(
            pickup=DEMO_JOB_PICKUP,
            dropoff=DEMO_JOB_DROPOFF,
            trigger_time=DEMO_JOB_TIME_SECONDS,
            priority=DEMO_JOB_PRIORITY,
        )
    )

mission_manager = JobMissionManager(
    service_points=SERVICE_POINTS,
    planner=planner,
    default_pickup=initial_pickup,
    default_dropoff=initial_dropoff,
)

planned_distance = 0.0
current_node = BASE_NODE
active_follower = None
active_leg = None
active_leg_target = None

visible_args = [arg for arg in sys.argv[1:] if arg.strip()]
print(f"Controller args: {', '.join(visible_args) if visible_args else 'none'}")
print(f"Initial delivery job: {initial_pickup}->{initial_dropoff}")

if scheduled_jobs:
    for job in scheduled_jobs:
        print(
            "Scheduled delivery job: "
            f"{job.label()} at {job.trigger_time:.1f}s, priority={job.priority}"
        )
else:
    print("Scheduled delivery job: none")

state = RobotState.IDLE
previous_state = None

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


def route_distance(route):
    total = 0.0

    for index in range(len(route) - 1):
        total += planner.distance_between(route[index], route[index + 1])

    return total


def arrival_threshold_for(node):
    if node == BASE_NODE:
        return BASE_ARRIVAL_THRESHOLD

    return DELIVERY_ARRIVAL_THRESHOLD


def activate_named_route(start, target, final_threshold, current_time, leg):
    global active_follower, active_leg, active_leg_target, planned_distance

    route = planner.plan(start, target)

    if len(route) < 2:
        active_follower = None
        active_leg = leg
        active_leg_target = target
        return route

    active_follower = create_follower(route, final_threshold)
    active_leg = leg
    active_leg_target = target
    planned_distance += route_distance(route)
    reset_progress_watch(current_time)

    return route


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


def print_job_decision(current_time, decision):
    action = decision["action"]
    job_label = decision.get("job")

    if action == "reject":
        print(f"Incoming job at {current_time:.1f}s rejected: {job_label} ({decision['reason']})")
        return

    if action in ["merge_current", "merge_pending"]:
        print(
            f"Incoming job at {current_time:.1f}s merged: "
            f"{job_label}, priority={decision.get('priority', 0)}"
        )
        return

    if action == "accept":
        print(
            f"Incoming job at {current_time:.1f}s accepted: "
            f"{job_label}, priority={decision.get('priority', 0)}"
        )
        return

    print(
        f"Incoming job at {current_time:.1f}s queued: "
        f"{job_label}, priority={decision.get('priority', 0)}"
    )


def process_scheduled_jobs(current_time, state):
    if state in [RobotState.FINISHED, RobotState.ERROR]:
        return

    for job in scheduled_jobs:
        if job.dispatched or current_time < job.trigger_time:
            continue

        job.dispatched = True
        decision = mission_manager.handle_incoming_job(
            pickup=job.pickup,
            dropoff=job.dropoff,
            current_time=current_time,
            priority=job.priority,
        )
        print_job_decision(current_time, decision)


def begin_current_job(current_time):
    global current_node, state

    job = mission_manager.current_job()

    if job is None:
        begin_return_to_base(current_time)
        return

    if current_node == job.pickup:
        print(f"Picked up package at {job.pickup} for {job.dropoff}")
        begin_dropoff_leg(job, current_time)
        return

    route = activate_named_route(
        current_node,
        job.pickup,
        arrival_threshold_for(job.pickup),
        current_time,
        LEG_PICKUP,
    )
    state = RobotState.GO_TO_DESTINATION
    print(f"State: GO_TO_PICKUP -> {job.pickup}")
    print(f"A* route to pickup: {' -> '.join(route)}")


def begin_dropoff_leg(job, current_time):
    global state

    route = activate_named_route(
        job.pickup,
        job.dropoff,
        arrival_threshold_for(job.dropoff),
        current_time,
        LEG_DROPOFF,
    )
    state = RobotState.GO_TO_DESTINATION
    print(f"State: GO_TO_DROPOFF -> {job.dropoff}")
    print(f"A* route to dropoff: {' -> '.join(route)}")


def begin_return_to_base(current_time):
    global state

    if current_node == BASE_NODE:
        state = RobotState.FINISHED
        return

    route = activate_named_route(
        current_node,
        BASE_NODE,
        BASE_ARRIVAL_THRESHOLD,
        current_time,
        LEG_RETURN_BASE,
    )
    state = RobotState.RETURN_TO_BASE
    print("State: RETURN_TO_BASE")
    print(f"A* route to base: {' -> '.join(route)}")


def finish_active_leg(current_time):
    global current_node, state

    movement.stop()
    current_node = active_leg_target
    metrics.record_waypoint(current_node)

    if active_leg == LEG_PICKUP:
        job = mission_manager.current_job()
        print(f"Reached pickup point: {current_node}")
        print(f"Picked up package at {current_node} for {job.dropoff}")
        begin_dropoff_leg(job, current_time)
        return

    if active_leg == LEG_DROPOFF:
        job = mission_manager.current_job()
        print(f"Reached dropoff point: {current_node}")
        print(f"Package delivered: {job.pickup}->{job.dropoff}")
        metrics.record_delivery(job.pickup, job.dropoff)
        mission_manager.mark_current_delivered()

        next_job = mission_manager.select_next_job(current_node)

        if next_job is not None:
            print(f"Next queued job selected: {next_job.label()}")
            begin_current_job(current_time)
        else:
            begin_return_to_base(current_time)

        return

    if active_leg == LEG_RETURN_BASE:
        print("Reached base")
        next_job = mission_manager.select_next_job(current_node)

        if next_job is not None:
            print(f"Next queued job selected: {next_job.label()}")
            begin_current_job(current_time)
        else:
            state = RobotState.FINISHED


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

    process_scheduled_jobs(current_time, state)

    if state == RobotState.IDLE:
        print("State: IDLE")
        metrics.start_mission()
        mission_start_time = current_time
        print("Mission model: fixed pickup/dropoff jobs over BASE, POINT_A, POINT_B")
        print("Tracking mode: pure pursuit lookahead")
        print(f"Obstacle avoidance enabled: {obstacle_avoidance_enabled}")
        begin_current_job(current_time)

    elif state == RobotState.GO_TO_DESTINATION:
        result = active_follower.update(pose)
        record_passed_nodes(result)

        if current_time - last_status_print > 2.0:
            print_tracking_status(f"Tracking to {active_leg_target}", result)
            last_status_print = current_time

        if result["done"]:
            finish_active_leg(current_time)

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

    elif state == RobotState.RETURN_TO_BASE:
        result = active_follower.update(pose)
        record_passed_nodes(result)

        if current_time - last_status_print > 2.0:
            print_tracking_status("Tracking to BASE", result)
            last_status_print = current_time

        if result["done"]:
            finish_active_leg(current_time)

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
