import math


WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class Odometry:
    def __init__(self, robot, timestep, initial_pose):
        self.robot = robot
        self.x, self.y, self.theta = initial_pose
        self.path_length = 0.0
        self.last_time = robot.getTime()

        self.left_sensor = self._get_position_sensor("left wheel sensor", timestep)
        self.right_sensor = self._get_position_sensor("right wheel sensor", timestep)
        self.last_left_position = None
        self.last_right_position = None

    def _get_position_sensor(self, name, timestep):
        try:
            sensor = self.robot.getDevice(name)
            sensor.enable(timestep)
            return sensor
        except Exception:
            print(f"Warning: position sensor {name} not found")
            return None

    def update(self, wheel_speeds):
        current_time = self.robot.getTime()
        dt = max(0.0, current_time - self.last_time)
        self.last_time = current_time

        if self.left_sensor is not None and self.right_sensor is not None:
            left_position = self.left_sensor.getValue()
            right_position = self.right_sensor.getValue()

            if self.last_left_position is None or self.last_right_position is None:
                self.last_left_position = left_position
                self.last_right_position = right_position
                return self.pose()

            left_delta = left_position - self.last_left_position
            right_delta = right_position - self.last_right_position

            self.last_left_position = left_position
            self.last_right_position = right_position
        else:
            left_speed, right_speed = wheel_speeds
            left_delta = left_speed * dt
            right_delta = right_speed * dt

        left_distance = left_delta * WHEEL_RADIUS
        right_distance = right_delta * WHEEL_RADIUS

        center_distance = (left_distance + right_distance) / 2.0
        theta_delta = (right_distance - left_distance) / AXLE_LENGTH
        theta_mid = self.theta + theta_delta / 2.0

        self.x += center_distance * math.cos(theta_mid)
        self.y += center_distance * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + theta_delta)
        self.path_length += abs(center_distance)

        return self.pose()

    def pose(self):
        return self.x, self.y, self.theta
