from archemist.util.location import Location
from enum import Enum
from archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError
from datetime import datetime

class RobotState(Enum):
    EXECUTING_JOB = 0
    WAITING_ON_STATION = 1
    IDLE = 2

class RobotOutputDescriptor:
    def __init__(self):
        self._success = False
        self._has_result = False
        self._success = False
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

    def addTimeStamp(self):
        self._timestamp = datetime.now()

class RobotOpDescriptor:
    def __init__(self, output: RobotOutputDescriptor):
        self._output = output
        self._timestamp = None

    @property
    def output(self):
        return self._output

    def addTimeStamp(self):
        self._timestamp = datetime.now()

class VialMoveOpDescriptor(RobotOpDescriptor):
    def __init__(self, start_pos: Location, end_pos: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._start_pos = start_pos
        self._end_pos = end_pos

    @property
    def start_pos(self):
        return self._start_pos

    @property
    def end_pos(self):
        return self._end_pos

class RackMoveOpDescriptor(RobotOpDescriptor):
    def __init__(self, start_pos: Location, end_pos: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._start_pos = start_pos
        self._end_pos = end_pos

    @property
    def start_pos(self):
        return self._start_pos

    @property
    def end_pos(self):
        return self._end_pos

class TransportBatchOpDescriptor(RobotOpDescriptor):
    def __init__(self, target_loc: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._target_loc = target_loc

    @property
    def target_loc(self):
        return self._target_loc

class SpecialJobOpDescriptor(RobotOpDescriptor):
    def __init__(self, job_name: str, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._job_name = job_name

    @property
    def job_name(self):
        return self._job_name

class Robot:
    def __init__(self, id: int):
        self._id = id

        self._available = False
        self._operational = False
        self._location = None
        
        self._assigned_job = None
        self._complete_job = None

        self._state = RobotState.IDLE
        
        self._current_robot_op = None
        self._robot_op_history = []

    @property
    def id(self):
        return self._id

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
    def robot_op_history(self):
        return self._robot_op_history

    def set_robot_op(self, robot_op: RobotOpDescriptor):
        self._current_robot_op = robot_op
        self._robot_op_history = self._robot_op_history.append(robot_op)

    def assign_job(self, object):
        if(self._assigned_job is None):
            self._assigned_job = object
            self._state = RobotState.EXECUTING_JOB
            # print('Batch {id} assigned to robot {name}'.format(id=object.id,
            #       name=self.__class__.__name__))
        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def job_completed(self):
        self._complete_job = self._assigned_job
        self._assigned_job = None

    def has_complete_job(self):
        return self._complete_job is not None

    def get_complete_job(self):
        obj = self._complete_job
        if self._complete_job is not None: 
            self._complete_job = None
            self._state = RobotState.IDLE
        return obj

class mobileRobot(Robot):
    def __init__(self, id: int):
        super().__init__(id)
        self._rack_holders = []

    @property 
    def rack_holders(self):
        return self._rack_holders

class armRobot(Robot):
    def __init__(self, id: int):
        super().__init__(id)
        self._pose = None

    @property
    def pose(self):
        return self._pose

    def moveToPose(self, pose: str):
        self._pose = pose
