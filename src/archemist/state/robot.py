from archemist.state.station import State
from archemist.util.location import Location
from archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError
from datetime import datetime

class RobotOutputDescriptor:
    def __init__(self, opName: str):
        self._opName = opName
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

    @property
    def opName(self):
        return self._opName

    def addTimeStamp(self):
        self._timestamp = datetime.now()

class RobotOpDescriptor:
    def __init__(self, robotName: str, output: RobotOutputDescriptor):
        self._robotName = robotName
        self._output = output
        self._timestamp = None

    @property
    def robotName(self):
        return self._robotName

    @property
    def output(self):
        return self._output

    def addTimeStamp(self):
        self._timestamp = datetime.now()

class VialMoveOpDescriptor(RobotOpDescriptor):
    def __init__(self, robotName, start_pos: Location, end_pos: Location, output: RobotOutputDescriptor):
        super().__init__(robotName=robotName, output=output)
        self._start_pos = start_pos
        self._end_pos = end_pos

    @property
    def start_pos(self):
        return self._start_pos

    @property
    def end_pos(self):
        return self._end_pos

class RackPlaceOpDescriptor(RobotOpDescriptor):
    def __init__(self, robotName, end_pos: Location, output: RobotOutputDescriptor):
        super().__init__(robotName=robotName, output=output)
        self._end_pos = end_pos

    @property
    def end_pos(self):
        return self._end_pos

class RackPickOpDescriptor(RobotOpDescriptor):
    def __init__(self, robotName, start_pos: Location, output: RobotOutputDescriptor):
        super().__init__(robotName=robotName, output=output)
        self._start_pos = start_pos

    @property
    def start_pos(self):
        return self._start_pos

class TransportBatchOpDescriptor(RobotOpDescriptor):
    def __init__(self, robotName, target_loc: Location, output: RobotOutputDescriptor):
        super().__init__(robotName=robotName, output=output)
        self._target_loc = target_loc

    @property
    def target_loc(self):
        return self._target_loc



class Robot:
    def __init__(self, id: int):
        self._id = id

        self._available = False
        self._operational = False
        self._location = None
        
        self._assigned_batch = None
        self._processed_batch = None
        self._state = State.IDLE
        
        self._currentRobotOp = None
        self._robotOpHistory = []

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

    @state.setter
    def state(self, value):
        if isinstance(value, State):
            self._state = value
        else:
            raise ValueError

    def setRobotOp(self, robotOp: RobotOpDescriptor):
        self._currentRobotOp = robotOp
        self._robotOpHistory = self._robotOpHistory.append(robotOp)

    def add_batch(self, object):
        if(self._assigned_batch is None):
            self._assigned_batch = object
            # print('Batch {id} assigned to robot {name}'.format(id=object.id,
            #       name=self.__class__.__name__))
        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def has_processed_batch(self):
        return self._processed_batch is not None

    def get_processed_batch(self):
        obj = self._processed_batch
        if self._processed_batch is not None: 
            self._processed_batch = None
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
