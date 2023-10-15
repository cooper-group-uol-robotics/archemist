from datetime import datetime
from typing import Any, List, Union, Dict, Type
from bson.objectid import ObjectId
from archemist.core.persistence.object_factory import StationOpFactory
from archemist.core.models.batch_model import SampleModel,BatchModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.persistence.models_proxy import EmbedModelProxy, ModelProxy, ListProxy
from archemist.core.util import Location

class Sample:
    def __init__(self, sample_model: Union[SampleModel, EmbedModelProxy]):
        self._model_proxy = sample_model

    @property
    def materials(self) -> List[Dict]:
        return self._model_proxy.materials

    @property
    def details(self) -> Dict:
        return self._model_proxy.details

    @details.setter
    def details(self, new_details: Dict):
        self._model_proxy.details = new_details

    @property
    def station_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.station_ops, StationOpFactory.create_from_model)

    def add_station_op(self, station_op: Any):
        self._model_proxy.station_ops.append(station_op.model)

    def add_material(self, material_name: str, material_id: int, amount: float, unit: str):
        added_material_dict = {
            "name": material_name,
            "id": material_id,
            "amount": amount,
            "unit": unit
        }
        self._model_proxy.materials.append(added_material_dict)

class Batch:
    def __init__(self, batch_model: Union[BatchModel, ModelProxy]) -> None:
        if isinstance(batch_model, ModelProxy):
            self._model_proxy = batch_model
        else:
            self._model_proxy = ModelProxy(batch_model)

    @classmethod
    def from_args(cls, num_samples: int, location:Location=None):
        model = BatchModel()
        model.location = location.to_dict() if location else {}
        model.samples.extend([SampleModel() for _ in range(num_samples)])
        model.save()
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
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._model_proxy.location = location.to_dict()
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

