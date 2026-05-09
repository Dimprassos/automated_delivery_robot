class MissionMetrics:
    def __init__(self, robot):
        self.robot = robot
        self.start_time = None
        self.end_time = None
        self.obstacle_avoidance_count = 0
        self.success = False

    def start_mission(self):
        self.start_time = self.robot.getTime()
        print("Mission started")

    def count_obstacle_avoidance(self):
        self.obstacle_avoidance_count += 1

    def finish_mission(self, success=True):
        self.end_time = self.robot.getTime()
        self.success = success

        mission_time = 0.0

        if self.start_time is not None:
            mission_time = self.end_time - self.start_time

        print("\n===== Mission Metrics =====")
        print(f"Success: {self.success}")
        print(f"Mission time: {mission_time:.2f} seconds")
        print(f"Obstacle avoidance actions: {self.obstacle_avoidance_count}")
        print("===========================\n")