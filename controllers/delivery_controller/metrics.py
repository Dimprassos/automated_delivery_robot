class MissionMetrics:
    def __init__(self, robot):
        self.robot = robot
        self.start_time = None
        self.end_time = None
        self.obstacle_avoidance_count = 0
        self.visited_waypoints = []
        self.delivered_to = None
        self.deliveries = []
        self.path_length = 0.0
        self.planned_distance = 0.0
        self.failure_reason = None
        self.success = False

    def start_mission(self):
        self.start_time = self.robot.getTime()
        print("Mission started")

    def count_obstacle_avoidance(self):
        self.obstacle_avoidance_count += 1

    def record_waypoint(self, waypoint_name):
        self.visited_waypoints.append((waypoint_name, self.robot.getTime()))

    def record_delivery(self, pickup, dropoff=None):
        if dropoff is None:
            dropoff = pickup
            pickup = None

        self.delivered_to = dropoff
        self.deliveries.append((pickup, dropoff, self.robot.getTime()))

    def finish_mission(
        self,
        success=True,
        path_length=0.0,
        planned_distance=0.0,
        failure_reason=None,
    ):
        self.end_time = self.robot.getTime()
        self.success = success
        self.path_length = path_length
        self.planned_distance = planned_distance
        self.failure_reason = failure_reason

        mission_time = 0.0

        if self.start_time is not None:
            mission_time = self.end_time - self.start_time

        efficiency = 0.0

        if self.path_length > 0.0 and self.planned_distance > 0.0:
            efficiency = self.planned_distance / self.path_length

        print("\n===== Mission Metrics =====")
        print(f"Success: {self.success}")
        if self.deliveries:
            delivered_jobs = []

            for pickup, dropoff, _ in self.deliveries:
                if pickup is None:
                    delivered_jobs.append(dropoff)
                else:
                    delivered_jobs.append(f"{pickup}->{dropoff}")

            print(f"Delivered jobs: {', '.join(delivered_jobs)}")
        else:
            print(f"Delivered to: {self.delivered_to}")
        print(f"Mission time: {mission_time:.2f} seconds")
        print(f"Obstacle avoidance actions: {self.obstacle_avoidance_count}")
        print(f"Estimated path length: {self.path_length:.2f} meters")
        print(f"Planned route distance: {self.planned_distance:.2f} meters")
        print(f"Route efficiency: {efficiency:.2f}")

        if self.failure_reason:
            print(f"Failure reason: {self.failure_reason}")

        if self.visited_waypoints:
            names = [name for name, _ in self.visited_waypoints]
            print(f"Visited waypoints: {', '.join(names)}")

        print("===========================\n")
