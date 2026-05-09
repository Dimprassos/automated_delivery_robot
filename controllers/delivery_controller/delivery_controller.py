from controller import Robot

from fsm import RobotState
from movement import MovementController
from sensors import SensorController
from metrics import MissionMetrics


print("Delivery controller is running")

robot = Robot()
timestep = int(robot.getBasicTimeStep())

movement = MovementController(robot)
sensors = SensorController(robot, timestep)
metrics = MissionMetrics(robot)

state = RobotState.IDLE
previous_state = None

mission_start_time = None
avoidance_start_time = None
avoidance_phase = 0
avoidance_direction = "right"

metrics_printed = False


while robot.step(timestep) != -1:
    current_time = robot.getTime()

    if state == RobotState.IDLE:
        print("State: IDLE")
        metrics.start_mission()
        mission_start_time = current_time
        state = RobotState.GO_TO_DESTINATION

    elif state == RobotState.GO_TO_DESTINATION:
        movement.move_forward(speed=3.0)

        if sensors.obstacle_detected_front():
            print("Obstacle detected - switching to AVOID_OBSTACLE")
            metrics.count_obstacle_avoidance()

            previous_state = state
            avoidance_start_time = current_time
            avoidance_phase = 0

            # Αν το εμπόδιο είναι πιο πολύ αριστερά, στρίβουμε δεξιά.
            # Αν είναι πιο πολύ δεξιά, στρίβουμε αριστερά.
            if sensors.obstacle_more_on_left():
                avoidance_direction = "right"
            else:
                avoidance_direction = "left"

            state = RobotState.AVOID_OBSTACLE

        elif current_time - mission_start_time > 14:
            movement.stop()
            state = RobotState.DELIVER

    elif state == RobotState.AVOID_OBSTACLE:
        elapsed = current_time - avoidance_start_time

        # Phase 0: κάνει λίγο πίσω για να ξεκολλήσει από το εμπόδιο
        if avoidance_phase == 0:
            movement.move_backward(speed=2.0)

            if elapsed > 0.5:
                avoidance_phase = 1
                avoidance_start_time = current_time

        # Phase 1: στρίβει μακριά από το εμπόδιο
        elif avoidance_phase == 1:
            if avoidance_direction == "right":
                movement.turn_right(speed=2.5)
            else:
                movement.turn_left(speed=2.5)

            if elapsed > 1.2:
                avoidance_phase = 2
                avoidance_start_time = current_time

        # Phase 2: κινείται μπροστά για να περάσει δίπλα από το εμπόδιο
        elif avoidance_phase == 2:
            movement.move_forward(speed=2.5)

            if elapsed > 1.8:
                avoidance_phase = 3
                avoidance_start_time = current_time

        # Phase 3: προσπαθεί να επανέλθει στην αρχική πορεία
        elif avoidance_phase == 3:
            if avoidance_direction == "right":
                movement.turn_left(speed=2.0)
            else:
                movement.turn_right(speed=2.0)

            if elapsed > 0.8:
                avoidance_phase = 4
                avoidance_start_time = current_time

        # Phase 4: συνεχίζει μόνο όταν καθαρίσει ο μπροστινός δρόμος
        elif avoidance_phase == 4:
            movement.move_forward(speed=2.0)

            if sensors.path_clear_front():
                print("Path is clear - returning to GO_TO_DESTINATION")
                movement.stop()
                state = previous_state

            # Safety timeout για να μη μείνει για πάντα στο avoidance
            elif elapsed > 3.0:
                print("Avoidance timeout - returning to GO_TO_DESTINATION")
                movement.stop()
                state = previous_state

    elif state == RobotState.DELIVER:
        print("State: DELIVER")
        print("Package delivered")

        state = RobotState.RETURN_TO_BASE
        mission_start_time = current_time

    elif state == RobotState.RETURN_TO_BASE:
        movement.move_backward(speed=3.0)

        if current_time - mission_start_time > 5:
            movement.stop()
            state = RobotState.FINISHED

    elif state == RobotState.FINISHED:
        movement.stop()

        if not metrics_printed:
            print("State: FINISHED")
            metrics.finish_mission(success=True)
            metrics_printed = True