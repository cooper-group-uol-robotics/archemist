from enum import Enum, auto

class RobotState(Enum):
    INACTIVE = auto()
    ACTIVE = auto()
    ERROR = auto()

class RobotTaskType(Enum):
    LOAD_TO_ROBOT = auto()
    UNLOAD_FROM_ROBOT = auto()
    MANIPULATION = auto()
    OTHER = auto()

class MobileRobotMode(Enum):
    OPERATIONAL = auto()
    COOLDOWN = auto()
    MAINTENANCE = auto()

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

class OpResult(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    SKIPPED = auto()

class ProcessStatus(Enum):
    INACTIVE = auto()
    RUNNING = auto()
    WAITING_ON_ROBOT_OPS = auto()
    WAITING_ON_STATION_OPS = auto()
    WAITING_ON_STATION_PROCS = auto()
    FINISHED = auto()

class TimeUnit(Enum):
    SECONDS = auto()
    MINUTES = auto()
    HOURS = auto()