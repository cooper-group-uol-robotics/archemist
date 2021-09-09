from archemist.exceptions import exception
from enum import Enum

class State(Enum):
    WAITING = 0
    EXECUTING = 1
    FINISHED = 2
    IDLE = 3


class Location:
    def __init__(self, name: str, node_id: int, graph_id: int, map_id: int, desk_pos: str):
        self._name = name
        self._node_id = node_id
        self._graph_id = graph_id
        self._map_id = map_id
        self._desk_pos = desk_pos

    @property
    def name(self):
        return self._name

    @property
    def node_id(self):
        return self._node_id

    @property
    def graph_id(self):
        return self._graph_id

    @property
    def map_id(self):
        return self._map_id

class StationOutputDescriptor:
    def __init__(self, opName: str, success:bool):
        self._opName = opName
        self._success = success

    @property
    def success(self):
        return self._success

    @property
    def opName(self):
        return self._opName

class StationOpDescriptor:
    def __init__(self, stationName: str):
        self._stationName = stationName

    @property
    def stationName(self):
        return self._stationName


class Station:
    def __init__(self, id: int, location: Location):
        self._id = id
        self._location = location
        self._available = False
        self._operational = False
        self._assigned_batch = None
        self._state = State.IDLE
        self._currentStationOp = None
        self._currentStationResult = None
        self._stationOpHistory = []
        self._stationOutputHistory = []



    @property
    def location(self):
        return self._location

    def setStationOp(self, stationOp: StationOpDescriptor):
        self._currentStationOp = stationOp
        self._stationOpHistory = self._stationOpHistory.append(stationOp)

    def setOperationResult(self, opResult: StationOutputDescriptor):
        self._currentStationResult = opResult
        self._stationOutputHistory = self._stationOutputHistory.append(opResult)

    def getOperationResult(self):
        return self._currentStationResult

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

    def add_batch(self, batch):
        if(self._assigned_batch is None):
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
