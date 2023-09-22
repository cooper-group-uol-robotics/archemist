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
    INACTIVE = auto()
    ACTIVE = auto()
    ERROR = auto()

class OpState(Enum):
    INVALID = auto()
    ASSIGNED = auto()
    EXECUTING = auto()
    TO_BE_REPEATED = auto()
    TO_BE_SKIPPED = auto()

class TimeUnit(Enum):
    SECONDS = auto()
    MINUTES = auto()
    HOURS = auto()