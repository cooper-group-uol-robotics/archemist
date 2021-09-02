from src.archemist.exceptions import exception


class Location:
    def __init__(self, name: str, node_id: int, graph_id: int, map_id: int):
        self._name = name
        self._node_id = node_id
        self._graph_id = graph_id
        self._map_id = map_id

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
    def __init__(self, success:bool):
        self._success = success
    
    @property
    def success(self):
        return self._success

class StationOpDescriptor:
    def __init__(self, name: str, id: int):
        self._name = name
        self._id = id

    def complete(self):
        out = StationOutputDescriptor(True)
        return out

    @property
    def name(self):
        return self._name
    
    @property
    def id(self):
        return self._id


class Station:
    def __init__(self, name: str, id: int, location: Location):
        self._name = name
        self._type = "none"
        self._id = id
        self._location = location
        self._available = False
        self._operational = False
        self._assigned_batch = None
        self._finished = True
        self._result = None


    @property
    def location(self):
        return self._location
    
    def setStationOp(self, stationOp: StationOpDescriptor):
        self._stationOp = stationOp
    
    def getResult(self):
        return self._stationOp.complete()

    @property
    def finished(self):
            return self._finished

    @finished.setter
    def finished(self, value):
        if isinstance(value, bool):
            self._finished = value
        else:
            raise ValueError

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

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


