from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.util.location import Location
from archemist.state.batch import Batch, Sample
class State(Enum):
    WAITING = 0
    EXECUTING = 1
    IDLE = 2

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
    def __init__(self, id: int, rack_holder: Location, pre_load: Location,
                 load: Location, post_load: Location):
        self._id = id
        self._rack_holder = rack_holder
        self._pre_load = pre_load
        self._load = load
        self._post_load = post_load

        self._available = False
        self._operational = False

        self._assigned_batch = None
        self._processed_batch = None
        self._state = State.IDLE
        
        self._currentStationOp = None
        self._stationOpHistory = []



    @property
    def rack_holder_loc(self):
        return self._rack_holder

    @property
    def pre_load_loc(self):
        return self._pre_load

    @property
    def load_loc(self):
        return self._load

    @property
    def post_load_loc(self):
        return self._post_load

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
            self._assigned_batch = batch
            print('Sample {id} assigned to station {name}'.format(id=batch.id,
                  name=self._name))
        else:
            raise exception.StationAssignedRackError(self._name)

    def has_processed_batch(self):
        return self._processed_batch != None

    def get_processed_batch(self):
        sample = self._processed_batch
        if not self._processed_batch: 
            self._processed_batch = None
        return sample
