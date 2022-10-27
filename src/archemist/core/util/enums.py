from enum import Enum, auto

class RobotState(Enum):
    OP_ASSIGNED = auto()
    SKIP_OP = auto()
    REPEAT_OP = auto()
    EXECUTING_OP = auto()
    EXECUTION_COMPLETE = auto()
    IDLE = auto()

class RobotTaskType(Enum):
    LOAD_TO_ROBOT = auto()
    UNLOAD_FROM_ROBOT = auto()
    MANIPULATION = auto()
    OTHER = auto()

class StationState(Enum):
    IDLE = auto()
    PROCESSING = auto()
    WAITING_ON_ROBOT = auto()
    OP_ASSIGNED = auto()
    SKIP_OP = auto()
    REPEAT_OP = auto()
    EXECUTING_OP = auto()
    OP_COMPLETE = auto()