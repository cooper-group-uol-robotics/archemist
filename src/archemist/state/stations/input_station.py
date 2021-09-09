from archemist.state.station import Station, Location, StationOpDescriptor
from archemist.state.batch import Batch


class InputStation(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)
        self._batches = []

    def batches_available(self):
        return self._batches != []

    def add_batch(self, batch):
        self._batches.append(batch)
        if(self._assigned_batch is None):
            self._assigned_batch = self._batches.pop
            print('Batch {id} assigned to station {name}'.format(id=batch.id,
                  name=self._name))

    def retrieve_batch(self):
        ret_batch = self._assigned_batch
        if(self._batches is not []):
            self._assigned_batch = self._batches.pop
        else:
            self._assigned_batch = None
        return ret_batch

class InputStationPickupOp(StationOpDescriptor):
    def __init__(self):
        super().__init__(stationName=InputStation.__class__.__name__)


class InputStationPlaceOp(StationOpDescriptor):
    def __init__(self):
        super().__init__(stationName=InputStation.__class__.__name__)
