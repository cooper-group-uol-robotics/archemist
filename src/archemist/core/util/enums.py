from enum import Enum, auto

class RobotState(Enum):
    INACTIVE = auto()
    ACTIVE = auto()
    ERROR = auto()

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

class OpOutcome(Enum):
    SUCCEEDED = auto()
    FAILED = auto()
    SKIPPED = auto()

class ProcessStatus(Enum):
    INACTIVE = auto()
    RUNNING = auto()
    REQUESTING_ROBOT_OPS = auto()
    WAITING_ON_ROBOT_OPS = auto()
    REQUESTING_STATION_OPS = auto()
    WAITING_ON_STATION_OPS = auto()
    REQUESTING_STATION_PROCS = auto()
    WAITING_ON_STATION_PROCS = auto()
    FINISHED = auto()

class LotStatus(Enum):
    CREATED = auto()
    STANDBY = auto()
    ONBOARDING = auto()
    READY_FOR_COLLECTION = auto()
    IN_WORKFLOW = auto()
    IN_PROCESS = auto()
    OFFBOARDING = auto()
    NEED_REMOVAL = auto()
    FINISHED = auto()

class WorkflowManagerStatus(Enum):
    INVALID = auto()
    RUNNING = auto()
    PAUSED = auto()