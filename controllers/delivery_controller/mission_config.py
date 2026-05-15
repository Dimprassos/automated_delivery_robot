import math


INITIAL_POSE = (-1.40, -0.90, 0.0)

WAYPOINTS = {
    "BASE": (-1.40, -0.90),
    "POINT_B": (-0.20, -0.90),
    "CORRIDOR_EAST": (0.20, -0.90),
    "CORRIDOR_NORTH": (0.20, 0.25),
    "POINT_A": (1.20, 0.25),
}

DEFAULT_DELIVERY_POINT = "POINT_A"
OBSTACLE_AVOIDANCE_ENABLED = False

DELIVERY_ROUTES = {
    "POINT_A": [
        "CORRIDOR_EAST",
        "CORRIDOR_NORTH",
        "POINT_A",
    ],
    "POINT_B": [
        "POINT_B",
    ],
}

RETURN_ROUTES = {
    "POINT_A": [
        "CORRIDOR_NORTH",
        "CORRIDOR_EAST",
        "BASE",
    ],
    "POINT_B": [
        "BASE",
    ],
}

ARRIVAL_THRESHOLD = 0.18
MISSION_TIMEOUT_SECONDS = 120.0


def route_distance(route_names, start_name="BASE"):
    total = 0.0
    previous = WAYPOINTS[start_name]

    for name in route_names:
        current = WAYPOINTS[name]
        total += math.hypot(current[0] - previous[0], current[1] - previous[1])
        previous = current

    return total


def planned_route_distance(delivery_point):
    return route_distance(DELIVERY_ROUTES[delivery_point]) + route_distance(
        RETURN_ROUTES[delivery_point],
        start_name=delivery_point,
    )
