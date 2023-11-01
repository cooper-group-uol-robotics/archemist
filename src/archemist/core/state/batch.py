from datetime import datetime
from typing import List, Union
from bson.objectid import ObjectId
from archemist.core.models.batch_model import BatchModel
from archemist.core.state.sample import Sample
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.util import Location

class Batch:
    def __init__(self, batch_model: Union[BatchModel, ModelProxy]) -> None:
        if isinstance(batch_model, ModelProxy):
            self._model_proxy = batch_model
        else:
            self._model_proxy = ModelProxy(batch_model)

    @classmethod
    def from_args(cls, num_samples: int, location:Location=None):
        model = BatchModel()
        if location:
            model.location = location.model
        model.save()
        samples = []
        for _ in range(num_samples):
            sample = Sample.from_args(model.id)
            samples.append(sample.model)
        model.update(samples=samples)
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = BatchModel.objects.get(id=object_id)
        return cls(model)

    @property
    def model(self) -> BatchModel:
        return self._model_proxy.model

    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

    @property
    def parent_lot_id(self) -> ObjectId:
        return self._model_proxy.parent_lot_id
    
    @parent_lot_id.setter
    def parent_lot_id(self, lot_id: ObjectId):
        self._model_proxy.parent_lot_id = lot_id

    @property
    def location(self) -> Location:
        return Location(self._model_proxy.location)

    @location.setter
    def location(self, new_location: Location):
        if isinstance(new_location, Location):
            self._model_proxy.location = new_location.model
        else:
            raise ValueError
        
    @property
    def samples(self) -> List[Sample]:
        return ListProxy(self._model_proxy.samples, Sample)

    @property
    def num_samples(self) -> int:
        return len(self._model_proxy.samples)

    @property
    def station_stamps(self):
        return self._model_proxy.station_stamps

    def add_station_stamp(self, station_stamp: str):
        timed_stamp = f'{datetime.now()} , {station_stamp}'
        self._model_proxy.station_stamps.append(timed_stamp)
        self._log_batch(f'({station_stamp}) stamp is added.')

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.object_id}'
    
    def __eq__(self, other_batch) -> bool:
        return self.object_id == other_batch.object_id

