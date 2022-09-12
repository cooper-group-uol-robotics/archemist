from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from archemist.util.location import Location
from bson.objectid import ObjectId


class OutputStation(Station):
    def __init__(self, db_name: str, station_dict: dict,  
                 liquids: list, solids: list):

        super().__init__(db_name,station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    # TODO handle mutliple batches
    # def batches_available(self):
    #     return self._batches != []

    # def add_batch(self, batch):
    #     self._batches.append(batch)
    #     if(self._assigned_batch is None):
    #         self._assigned_batch = self._batches.pop
    #         print('Batch {id} assigned to station {name}'.format(id=batch.id,
    #               name=self._name))

    # def retrieve_batch(self):
    #     ret_batch = self._assigned_batch
    #     if(self._batches is not []):
    #         self._assigned_batch = self._batches.pop
    #     else:
    #         self._assigned_batch = None
    #     return ret_batch

class OutputStationPlaceOp(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=OutputStation.__class__.__name__, output=output)


class OutputStationResultDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()
