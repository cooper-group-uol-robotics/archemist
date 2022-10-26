from enum import Enum

class RobotState(Enum):
    OP_ASSIGNED = 0
    EXECUTING_OP = 1
    EXECUTION_COMPLETE = 2
    IDLE = 3

class RobotTaskType(Enum):
    LOAD_TO_ROBOT = 0
    UNLOAD_FROM_ROBOT = 1
    MANIPULATION = 2
    OTHER = 3

class StationState(Enum):
    IDLE = 0
    PROCESSING = 1
    WAITING_ON_ROBOT = 2
    OP_ASSIGNED = 3
    EXECUTING_OP = 4
    OP_COMPLETE = 5