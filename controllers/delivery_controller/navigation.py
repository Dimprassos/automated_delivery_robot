import math

from movement import MAX_SPEED
from odometry import normalize_angle


class WaypointNavigator:
    def __init__(self, movement, arrival_threshold):
        self.movement = movement
        self.arrival_threshold = arrival_threshold

    def get_target_error(self, pose, target):
        x, y, theta = pose
        target_x, target_y = target

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - theta)

        return distance, heading_error

    def drive_to(self, pose, target, arrival_threshold=None):
        if arrival_threshold is None:
            arrival_threshold = self.arrival_threshold

        distance, heading_error = self.get_target_error(pose, target)

        if distance <= arrival_threshold:
            self.movement.stop()
            return True

        if abs(heading_error) > 0.45:
            turn_speed = min(3.0, max(1.2, abs(heading_error) * 2.2))

            if heading_error > 0.0:
                self.movement.turn_left(turn_speed)
            else:
                self.movement.turn_right(turn_speed)

            return False

        self.drive_toward(pose, target)
        return False

    def drive_toward(self, pose, target):
        distance, heading_error = self.get_target_error(pose, target)

        if abs(heading_error) > 0.45:
            turn_speed = min(3.0, max(1.2, abs(heading_error) * 2.2))

            if heading_error > 0.0:
                self.movement.turn_left(turn_speed)
            else:
                self.movement.turn_right(turn_speed)

            return

        base_speed = min(5.2, max(0.8, distance * 4.8))
        correction = max(-1.4, min(1.4, heading_error * 3.0))

        left_speed = base_speed - correction
        right_speed = base_speed + correction

        self.movement.set_speed(
            max(-MAX_SPEED, min(MAX_SPEED, left_speed)),
            max(-MAX_SPEED, min(MAX_SPEED, right_speed)),
        )
