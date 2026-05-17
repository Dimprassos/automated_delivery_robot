import math

from movement import MAX_SPEED
from odometry import AXLE_LENGTH, WHEEL_RADIUS


class RoutePathFollower:
    def __init__(
        self,
        movement,
        route_names,
        waypoints,
        final_threshold,
        base_lookahead=0.24,
        corner_lookahead=0.10,
        cruise_speed=0.085,
        corner_speed=0.04,
        final_speed=0.035,
        route_end_threshold=0.20,
    ):
        if len(route_names) < 2:
            raise ValueError("A route must contain at least a start and goal node")

        self.movement = movement
        self.route_names = route_names
        self.points = [waypoints[name] for name in route_names]
        self.final_threshold = final_threshold
        self.base_lookahead = base_lookahead
        self.corner_lookahead = corner_lookahead
        self.cruise_speed = cruise_speed
        self.corner_speed = corner_speed
        self.final_speed = final_speed
        self.route_end_threshold = max(final_threshold, route_end_threshold)

        self.segment_lengths = []
        self.cumulative_distances = [0.0]

        for index in range(len(self.points) - 1):
            ax, ay = self.points[index]
            bx, by = self.points[index + 1]
            segment_length = math.hypot(bx - ax, by - ay)

            if segment_length <= 0.0:
                raise ValueError(f"Route has a zero-length segment at index {index}")

            self.segment_lengths.append(segment_length)
            self.cumulative_distances.append(self.cumulative_distances[-1] + segment_length)

        self.total_length = self.cumulative_distances[-1]
        self.progress = 0.0
        self.next_node_index = 1

    def update(self, pose):
        projected_progress, lateral_error = self._project_pose(pose)
        self.progress = max(self.progress, projected_progress)

        final_x, final_y = self.points[-1]
        final_distance = math.hypot(final_x - pose[0], final_y - pose[1])
        remaining = max(0.0, self.total_length - self.progress)
        passed_nodes = self._consume_passed_nodes(pose)

        reached_final_point = final_distance <= self.final_threshold
        reached_route_end = (
            remaining <= self.final_threshold
            and final_distance <= self.route_end_threshold
        )

        if reached_final_point or reached_route_end:
            self.movement.stop()
            return self._result(
                done=True,
                pose=pose,
                remaining=remaining,
                lateral_error=lateral_error,
                lookahead_distance=0.0,
                lookahead_point=(final_x, final_y),
                turn_radius=float("inf"),
                linear_speed=0.0,
                passed_nodes=passed_nodes,
                motion_mode="done",
            )

        lookahead_distance = min(self._adaptive_lookahead(), max(remaining, self.final_threshold))
        lookahead_progress = min(self.total_length, self.progress + lookahead_distance)

        if self.next_node_index < len(self.points) - 1:
            lookahead_progress = min(
                lookahead_progress,
                self.cumulative_distances[self.next_node_index],
            )

        lookahead_point = self._point_at(lookahead_progress)

        curvature, target_distance, heading_error = self._curvature_to_point(pose, lookahead_point)
        linear_speed = self._adaptive_speed(remaining)
        turn_radius = float("inf")
        motion_mode = "track"

        if abs(curvature) > 1e-6:
            turn_radius = 1.0 / curvature

        if abs(heading_error) > 0.80:
            motion_mode = "align"
            linear_speed = 0.0
            turn_radius = 0.0
            self._turn_in_place(heading_error)
        else:
            self._drive_curvature(curvature, target_distance, linear_speed)

        return self._result(
            done=False,
            pose=pose,
            remaining=remaining,
            lateral_error=lateral_error,
            lookahead_distance=lookahead_distance,
            lookahead_point=lookahead_point,
            turn_radius=turn_radius,
            linear_speed=linear_speed,
            passed_nodes=passed_nodes,
            motion_mode=motion_mode,
        )

    def _project_pose(self, pose):
        x, y, _ = pose
        best_progress = 0.0
        best_distance = float("inf")

        active_segment = max(0, min(self.next_node_index - 1, len(self.points) - 2))

        for index in [active_segment]:
            ax, ay = self.points[index]
            bx, by = self.points[index + 1]
            segment_x = bx - ax
            segment_y = by - ay
            segment_length_sq = segment_x * segment_x + segment_y * segment_y
            raw_t = ((x - ax) * segment_x + (y - ay) * segment_y) / segment_length_sq
            t = max(0.0, min(1.0, raw_t))
            projected_x = ax + t * segment_x
            projected_y = ay + t * segment_y
            distance = math.hypot(x - projected_x, y - projected_y)

            if distance < best_distance:
                best_distance = distance
                best_progress = self.cumulative_distances[index] + t * self.segment_lengths[index]

        return best_progress, best_distance

    def _consume_passed_nodes(self, pose):
        passed_nodes = []
        pass_radius = 0.16
        x, y, _ = pose

        while self.next_node_index < len(self.route_names) - 1:
            node_x, node_y = self.points[self.next_node_index]
            node_distance = math.hypot(node_x - x, node_y - y)

            if node_distance > pass_radius:
                break

            passed_nodes.append(self.route_names[self.next_node_index])
            self.next_node_index += 1

        return passed_nodes

    def _point_at(self, progress):
        progress = max(0.0, min(self.total_length, progress))

        for index, segment_length in enumerate(self.segment_lengths):
            start_progress = self.cumulative_distances[index]
            end_progress = self.cumulative_distances[index + 1]

            if progress <= end_progress or index == len(self.segment_lengths) - 1:
                t = (progress - start_progress) / segment_length
                ax, ay = self.points[index]
                bx, by = self.points[index + 1]

                return ax + t * (bx - ax), ay + t * (by - ay)

        return self.points[-1]

    def _segment_index_at_progress(self):
        return max(0, min(self.next_node_index - 1, len(self.segment_lengths) - 1))

    def _upcoming_corner(self):
        index = self._segment_index_at_progress()

        if index >= len(self.points) - 2:
            return 0.0, float("inf")

        ax, ay = self.points[index]
        bx, by = self.points[index + 1]
        cx, cy = self.points[index + 2]

        first_x = bx - ax
        first_y = by - ay
        second_x = cx - bx
        second_y = cy - by
        first_length = math.hypot(first_x, first_y)
        second_length = math.hypot(second_x, second_y)

        if first_length <= 0.0 or second_length <= 0.0:
            return 0.0, float("inf")

        dot = first_x * second_x + first_y * second_y
        cosine = max(-1.0, min(1.0, dot / (first_length * second_length)))
        angle = math.acos(cosine)
        distance_to_corner = max(0.0, self.cumulative_distances[index + 1] - self.progress)

        return angle, distance_to_corner

    def _adaptive_lookahead(self):
        angle, distance_to_corner = self._upcoming_corner()

        if angle < 0.45 or distance_to_corner > 0.45:
            return self.base_lookahead

        corner_factor = max(0.0, min(1.0, distance_to_corner / 0.45))
        return self.corner_lookahead + (self.base_lookahead - self.corner_lookahead) * corner_factor

    def _adaptive_speed(self, remaining):
        angle, distance_to_corner = self._upcoming_corner()
        speed = self.cruise_speed

        if angle >= 0.45 and distance_to_corner < 0.50:
            corner_factor = max(0.0, min(1.0, distance_to_corner / 0.50))
            speed = self.corner_speed + (self.cruise_speed - self.corner_speed) * corner_factor

        if remaining < 0.35:
            final_factor = max(0.0, min(1.0, remaining / 0.35))
            speed = min(speed, self.final_speed + (self.cruise_speed - self.final_speed) * final_factor)

        return speed

    def _curvature_to_point(self, pose, point):
        x, y, theta = pose
        target_x, target_y = point
        dx = target_x - x
        dy = target_y - y

        local_x = math.cos(theta) * dx + math.sin(theta) * dy
        local_y = -math.sin(theta) * dx + math.cos(theta) * dy
        target_distance = max(0.001, math.hypot(local_x, local_y))
        heading_error = math.atan2(local_y, local_x)

        if local_x < 0.02:
            curvature = 12.0 if local_y >= 0.0 else -12.0
        else:
            curvature = 2.0 * local_y / (target_distance * target_distance)

        return curvature, target_distance, heading_error

    def _turn_in_place(self, heading_error):
        turn_speed = min(2.2, max(0.9, abs(heading_error) * 1.3))

        if heading_error > 0.0:
            self.movement.turn_left(turn_speed)
        else:
            self.movement.turn_right(turn_speed)

    def _drive_curvature(self, curvature, target_distance, linear_speed):
        if target_distance < 0.02:
            self.movement.stop()
            return

        angular_speed = curvature * linear_speed
        left_linear_speed = linear_speed - angular_speed * AXLE_LENGTH / 2.0
        right_linear_speed = linear_speed + angular_speed * AXLE_LENGTH / 2.0

        left_motor_speed = left_linear_speed / WHEEL_RADIUS
        right_motor_speed = right_linear_speed / WHEEL_RADIUS
        largest_speed = max(abs(left_motor_speed), abs(right_motor_speed), 1.0)

        if largest_speed > MAX_SPEED:
            scale = MAX_SPEED / largest_speed
            left_motor_speed *= scale
            right_motor_speed *= scale

        self.movement.set_speed(left_motor_speed, right_motor_speed)

    def _result(
        self,
        done,
        pose,
        remaining,
        lateral_error,
        lookahead_distance,
        lookahead_point,
        turn_radius,
        linear_speed,
        passed_nodes,
        motion_mode,
    ):
        return {
            "done": done,
            "pose": pose,
            "remaining": remaining,
            "progress": self.progress,
            "lateral_error": lateral_error,
            "lookahead_distance": lookahead_distance,
            "lookahead_point": lookahead_point,
            "turn_radius": turn_radius,
            "linear_speed": linear_speed,
            "passed_nodes": passed_nodes,
            "motion_mode": motion_mode,
        }
