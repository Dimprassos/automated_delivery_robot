from controller import Supervisor


ROBOT_DEF_NAME = "DELIVERY_ROBOT"
START_TRANSLATION = [-2.25, -0.72, 0.0]
START_ROTATION = [0.0, 0.0, 1.0, 1.5708]
SCENE_START_TRANSLATIONS = {
    "ACADEMIC_WING_FLOORS": [0.0, 0.0, 0.0],
    "SERVICE_POINT_MARKERS": [0.0, 0.0, 0.0],
    "FLOOR_PLAN_TEXT": [0.0, 0.0, 0.0],
    "OUTER_WEST_WALL": [-3.05, 0.0, 0.23],
    "OUTER_EAST_WALL": [3.05, 0.0, 0.23],
    "OUTER_SOUTH_WALL": [0.0, -2.05, 0.23],
    "OUTER_NORTH_WALL": [0.0, 2.05, 0.23],
    "NORTH_CLASSROOM_SOUTH_WALL_1": [-2.45, 0.48, 0.23],
    "NORTH_CLASSROOM_SOUTH_WALL_2": [-0.98, 0.48, 0.23],
    "NORTH_LAB_SOUTH_WALL_1": [1.12, 0.48, 0.23],
    "NORTH_LAB_SOUTH_WALL_2": [2.65, 0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_1": [-2.78, -0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_2": [-1.64, -0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_3": [-0.6, -0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_4": [0.6, -0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_5": [1.55, -0.48, 0.23],
    "SOUTH_ROOM_NORTH_WALL_6": [2.75, -0.48, 0.23],
    "TOP_PARTITION_CLASSROOM_LOUNGE": [-0.55, 1.25, 0.23],
    "TOP_PARTITION_LOUNGE_LAB": [0.55, 1.25, 0.23],
    "BOTTOM_PARTITION_MAIL_OFFICE": [-1.5, -1.25, 0.23],
    "BOTTOM_PARTITION_OFFICE_CROSS": [-0.48, -1.25, 0.23],
    "BOTTOM_PARTITION_CROSS_STORAGE": [0.48, -1.25, 0.23],
    "BOTTOM_PARTITION_STORAGE_RESEARCH": [1.42, -1.25, 0.23],
    "ARCHITECTURAL_DETAILS": [0.0, 0.0, 0.0],
    "SIGNAGE": [0.0, 0.0, 0.0],
    "REALISTIC_DOORWAYS": [0.0, 0.0, 0.0],
    "MAILROOM_BASE_FURNITURE": [0.0, 0.0, 0.0],
    "FACULTY_OFFICE_FURNITURE": [0.0, 0.0, 0.0],
    "CLASSROOM_FURNITURE": [0.0, 0.0, 0.0],
    "TEACHING_LAB_FURNITURE": [0.0, 0.0, 0.0],
    "STUDY_AND_SUPPORT_FURNITURE": [0.0, 0.0, 0.0],
    "ROOM_OBJECT_COLLIDERS": [0.0, 0.0, 0.0],
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
