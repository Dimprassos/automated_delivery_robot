from controller import Supervisor


ROBOT_DEF_NAME = "DELIVERY_ROBOT"
START_TRANSLATION = [-1.4, -0.9, 0.0]
START_ROTATION = [0.0, 0.0, 1.0, 0.0]
SCENE_START_TRANSLATIONS = {
    "BASE_MARKER": [-1.4, -0.9, 0.006],
    "POINT_B_MARKER": [-0.2, -0.9, 0.006],
    "POINT_A_MARKER": [1.2, 0.25, 0.006],
    "OUTER_WEST_WALL": [-2.05, 0.0, 0.18],
    "OUTER_EAST_WALL": [2.05, 0.0, 0.18],
    "OUTER_SOUTH_WALL": [0.0, -1.55, 0.18],
    "OUTER_NORTH_WALL": [0.0, 1.55, 0.18],
    "HALL_SOUTH_WALL": [-0.65, -1.18, 0.18],
    "HALL_NORTH_WALL_LEFT": [-1.05, -0.62, 0.18],
    "VERTICAL_WEST_WALL": [-0.08, 0.05, 0.18],
    "VERTICAL_EAST_WALL": [0.48, -0.55, 0.18],
    "DELIVERY_SOUTH_WALL": [1.15, -0.05, 0.18],
    "DELIVERY_NORTH_WALL": [0.95, 0.55, 0.18],
    "CLASSROOM_DESKS": [-1.25, 0.82, 0.035],
    "LAB_BENCH": [1.15, 0.95, 0.04],
    "OFFICE_DESK": [-1.35, -1.05, 0.035],
    "OBSTACLE_CORRIDOR": [-0.65, -0.84, 0.09],
    "OBSTACLE_TURN": [0.36, -0.05, 0.09],
}


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node = supervisor.getFromDef(ROBOT_DEF_NAME)

if robot_node is None:
    print(f"Reset supervisor warning: {ROBOT_DEF_NAME} not found")
else:
    robot_node.getField("translation").setSFVec3f(START_TRANSLATION)
    robot_node.getField("rotation").setSFRotation(START_ROTATION)

    for def_name, translation in SCENE_START_TRANSLATIONS.items():
        node = supervisor.getFromDef(def_name)

        if node is not None:
            node.getField("translation").setSFVec3f(translation)

    supervisor.simulationResetPhysics()
    print(f"Reset supervisor: {ROBOT_DEF_NAME} placed at start")

while supervisor.step(timestep) != -1:
    pass
