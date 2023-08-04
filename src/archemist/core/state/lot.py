import uuid
from typing import Union, List
from archemist.core.models.lot_model import LotModel
from archemist.core.state.batch import Batch
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy

class Lot:
    def __init__(self, lot_model: Union[LotModel, ModelProxy]) -> None:
        if isinstance(lot_model, ModelProxy):
            self._model_proxy = lot_model
        else:
            self._model_proxy = ModelProxy(lot_model)

    @classmethod
    def from_args(cls, batches: List[Batch]):
        model = LotModel()
        model.uuid = uuid.uuid4()
        model.batches = [batch.model for batch in batches]
        model.save()
        return cls(model)
    
    @property
    def model(self) -> LotModel:
        return self._model_proxy.model
    
    @property
    def uuid(self) -> uuid.UUID:
        return self._model_proxy.uuid
    
    @property
    def batches(self) -> List[Batch]:
        return ListProxy(self._model_proxy.batches, Batch)
    
    @property
    def num_batches(self) -> int:
        return len(self._model_proxy.batches)
    
    def add_station_stamp(self, station_stamp: str):
        for batch in self.batches:
            batch.add_station_stamp(station_stamp)