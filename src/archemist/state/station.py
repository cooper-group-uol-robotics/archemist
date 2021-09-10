from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.util.location import Location
from archemist.state.batch import Batch
class State(Enum):
    WAITING = 0
    EXECUTING = 1
    FINISHED = 2
    IDLE = 3

class StationOutputDescriptor:
    def __init__(self, opName: str):
        self._opName = opName
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

class StationOpDescriptor:
    def __init__(self, stationName: str, output: StationOutputDescriptor):
        self._stationName = stationName
        self._output = output
        self._timestamp = None

    @property
    def stationName(self):
        return self._stationName

    @property
    def output(self):
        return self._output

    def addTimeStamp(self):
        self._timestamp = datetime.now()


class Station:
    def __init__(self, id: int, location: Location):
        self._id = id
        self._location = location
        self._available = False
        self._operational = False
        self._assigned_batch = None
        self._current_vial = None
        self._state = State.IDLE
        self._currentStationOp = None
        self._stationOpHistory = []
        self._stationOutputHistory = []



    @property
    def location(self):
        return self._location

    def setStationOp(self, stationOp: StationOpDescriptor):
        self._currentStationOp = stationOp
        self._stationOpHistory = self._stationOpHistory.append(stationOp)

    @property
    def state(self):
            return self._state

    @state.setter
    def state(self, value):
        if isinstance(value, State):
            self._state = value
        else:
            raise ValueError

    @property
    def id(self):
        return self._id

    @property
    def location(self):
        return self._location

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

    def add_batch(self, batch: Batch):
        if(self._assigned_batch is None):
            batch.state = BatchState.READY_FOR_PROCESSING
            batch.location = self.location
            batch.addStationStamp(self.__class__.__name__)
            self._assigned_batch = batch
            print('Batch {id} assigned to station {name}'.format(id=batch.id,
                  name=self._name))
        else:
            raise exception.StationAssignedRackError(self._name)

    def retrieve_batch(self):
        ret_batch = self._assigned_batch
        if(self._assigned_batch is not None):
            self._assigned_batch = None
        else:
            raise exception.StationUnAssignedRackError(self._name)
        return ret_batch

    def has_finished_batch(self):
        if (self._assigned_batch != None):
            if (self._assigned_batch.state == BatchState.PROCESING_DONE):
                return True
        return False
