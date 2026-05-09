class SensorController:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        self.distance_sensors = []

        # e-puck proximity sensors: ps0 έως ps7
        for i in range(8):
            sensor_name = f"ps{i}"

            try:
                sensor = robot.getDevice(sensor_name)
                sensor.enable(timestep)
                self.distance_sensors.append(sensor)
            except Exception:
                print(f"Warning: sensor {sensor_name} not found")

    def get_sensor_values(self):
        return [sensor.getValue() for sensor in self.distance_sensors]

    def get_front_values(self):
        values = self.get_sensor_values()

        if len(values) < 8:
            return []

        # Μπροστινοί αισθητήρες του e-puck
        # ps0, ps1 = front/right
        # ps6, ps7 = front/left
        return [values[0], values[1], values[6], values[7]]

    def max_front_value(self):
        front_values = self.get_front_values()

        if not front_values:
            return 0.0

        return max(front_values)

    def obstacle_detected_front(self, threshold=120.0):
        return self.max_front_value() > threshold

    def path_clear_front(self, threshold=70.0):
        return self.max_front_value() < threshold

    def obstacle_more_on_left(self):
        values = self.get_sensor_values()

        if len(values) < 8:
            return False

        left_side = values[6] + values[7]
        right_side = values[0] + values[1]

        return left_side > right_side

    def print_front_sensor_values(self):
        print("Front sensor values:", self.get_front_values())