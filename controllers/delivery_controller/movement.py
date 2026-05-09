MAX_SPEED = 6.28


class MovementController:
    def __init__(self, robot):
        self.robot = robot

        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.stop()

    def set_speed(self, left_speed, right_speed):
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def move_forward(self, speed=3.0):
        self.set_speed(speed, speed)

    def move_backward(self, speed=3.0):
        self.set_speed(-speed, -speed)

    def turn_left(self, speed=2.0):
        self.set_speed(-speed, speed)

    def turn_right(self, speed=2.0):
        self.set_speed(speed, -speed)

    def stop(self):
        self.set_speed(0.0, 0.0)