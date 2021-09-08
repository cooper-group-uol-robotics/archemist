from src.archemist.state.station import Location, State
from src.archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError

class RobotOutputDescriptor:
    def __init__(self, opName: str, success:bool):
        self._opName = opName
        self._success = success
    
    @property
    def success(self):
        return self._success

    @property
    def opName(self):
        return self._opName

class RobotOpDescriptor:
    def __init__(self, robotName: str):
        self._robotName = robotName

    @property
    def robotName(self):
        return self._robotName 


class robot:
    def __init__(self, id: int):
        self._id = id
        self._available = False
        self._operational = False
        self._assigned_batch = None
        self._state = State.IDLE
        self._currentRobotOp = None
        self._currentRobotResult = None
        self._robotOpHistory = []
        self._robotOutputHistory = []
    
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

    def setOperationResult(self, opResult: RobotOutputDescriptor):
        self._currentRobotResult = opResult
        self._robotOutputHistory = self._robotOutputHistory.append(opResult)
    
    def getOperationResult(self):
        return self._currentRobotResult

    def add_batch(self, batch):
        if(self._assigned_batch is None):
            self._assigned_batch = batch
            print('Batch {id} assigned to robot {name}'.format(id=batch.id,
                  name=self._name))
        else:
            raise RobotAssignedRackError(self._name)

    def retrieve_batch(self):
        ret_batch = self._assigned_batch
        if(self._assigned_batch is not None):
            self._assigned_batch = None
        else:
            raise RobotUnAssignedRackError(self._name)
        return ret_batch

class mobileRobot(robot):
    def __init__(self, id: int):
        super().__init__(id)
        self._location = None

class armRobot(robot):
    def __init__(self, id: int):
        super().__init__(id)