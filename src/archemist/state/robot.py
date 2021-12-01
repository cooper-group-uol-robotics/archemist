from archemist.util.location import Location
from enum import Enum
from archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError
from datetime import datetime

class RobotState(Enum):
    EXECUTING_JOB = 0
    EXECUTION_COMPLETE = 1
    #WAITING_ON_STATION = 1
    IDLE = 2

class RobotOutputDescriptor:
    def __init__(self):
        self._success = False
        self._has_result = False
        self._timestamp = None

    @property
    def success(self):
        return self._success

    @success.setter
    def success(self, value):
        if isinstance(value, bool):
            self._success = value
        else:
            raise ValueError

    @property
    def has_result(self):
        return self._has_result

    @has_result.setter
    def has_result(self, value):
        if isinstance(value, bool):
            self._has_result = value
        else:
            raise ValueError

    def add_timestamp(self):
        self._timestamp = datetime.now()

class RobotOpDescriptor:
    def __init__(self, output: RobotOutputDescriptor):
        self._output = output
        self._timestamp = None

    @property
    def output(self):
        return self._output

    def add_timestamp(self):
        self._timestamp = datetime.now()

class MoveSampleOp(RobotOpDescriptor):
    def __init__(self, sample_index: int , start_location: Location, target_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._sample_index = sample_index
        self._start_location = start_location
        self._target_location = target_location

    @property
    def sample_index(self):
        return self._sample_index

    @property
    def start_location(self):
        return self._start_location

    @property
    def target_location(self):
        return self._target_location

class PickAndPlaceBatchOp(RobotOpDescriptor):
    def __init__(self, pick_location: Location, place_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._pick_location = pick_location
        self._place_location = place_location

    @property
    def pick_location(self):
        return self._pick_location

    @property
    def place_location(self):
        return self._place_location

class PickBatchToDeckOp(RobotOpDescriptor):
    def __init__(self, pick_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._pick_location = pick_location

    @property
    def pick_location(self):
        return self._pick_location

class PlaceBatchFromDeckOp(RobotOpDescriptor):
    def __init__(self, place_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._place_location = place_location

    @property
    def place_location(self):
        return self._place_location

class SpecialJobOpDescriptor(RobotOpDescriptor):
    def __init__(self, job_name: str, target_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._job_name = job_name
        self._job_location = target_location

    @property
    def job_name(self):
        return self._job_name

    @property
    def job_location(self):
        return self._job_location

class Robot:
    def __init__(self, id: int, saved_frames: list):
        self._id = id
        self._saved_frames = saved_frames

        self._available = False
        self._operational = False
        self._location = None
        
        self._assigned_job = None
        self._complete_job = None

        self._state = RobotState.IDLE
        
        self._robot_job_history = []

    @property
    def id(self):
        return self._id

    @property
    def saved_frames(self):
        return self._saved_frames

    @property
    def available(self):
        return self._available

    @available.setter
    def available(self, value):
        if isinstance(value, bool):
            self._available = value
        else:
            raise ValueError

    @property
    def operational(self):
        return self._operational

    @operational.setter
    def operational(self, value):
        if isinstance(value, bool):
            self._operational = value
        else:
            raise ValueError

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, value):
        if isinstance(value, Location):
            self._location = value
        else:
            raise ValueError

    @property
    def state(self):
            return self._state

    @property
    def assigned_job(self):
            return self._assigned_job

    @property
    def robot_job_history(self):
        return self._robot_job_history

    def assign_job(self, object):
        if(self._assigned_job is None):
            self._assigned_job = object
            self._state = RobotState.EXECUTING_JOB
            self._logRobot(f'Job is assigned. Robot state is {self._state}')
        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def complete_assigned_job(self):
        self._complete_job = self._assigned_job
        self._assigned_job = None
        self._robot_job_history.append(self._complete_job)
        self._state = RobotState.EXECUTION_COMPLETE
        self._logRobot(f'The assigned job is complete. Robot state is {self._state}')

    def has_complete_job(self):
        return self._complete_job is not None

    def get_complete_job(self):
        obj = self._complete_job
        if self._complete_job is not None: 
            self._complete_job = None
            self._state = RobotState.IDLE
            self._logRobot(f'The finished job is retrieved. Robot state is {self._state}')
        return obj

    def _logRobot(self, message: str):
        print(f'Robot [{self.__class__.__name__}, {self._id}]: ' + message)

class mobileRobot(Robot):
    def __init__(self, id: int, saved_frames: list):
        super().__init__(id, saved_frames)
        self._rack_holders = []

    @property 
    def rack_holders(self):
        return self._rack_holders

class armRobot(Robot):
    def __init__(self, id: int, saved_frames: list):
        super().__init__(id, saved_frames)