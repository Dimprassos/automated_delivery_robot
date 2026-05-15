import sys
from pathlib import Path

from controller import Supervisor


sys.path.append(str(Path(__file__).resolve().parents[1] / "delivery_controller"))

from mission_config import INITIAL_POSE


ROBOT_DEF_NAME = "DELIVERY_ROBOT"
START_TRANSLATION = [INITIAL_POSE[0], INITIAL_POSE[1], 0.0]
START_ROTATION = [0.0, 0.0, 1.0, INITIAL_POSE[2]]


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node = supervisor.getFromDef(ROBOT_DEF_NAME)

if robot_node is None:
    print(f"Reset supervisor warning: {ROBOT_DEF_NAME} not found")
else:
    robot_node.getField("translation").setSFVec3f(START_TRANSLATION)
    robot_node.getField("rotation").setSFRotation(START_ROTATION)
    supervisor.simulationResetPhysics()
    print(f"Reset supervisor: {ROBOT_DEF_NAME} placed at start")

while supervisor.step(timestep) != -1:
    pass
