from typing import Union, List
from bson.objectid import ObjectId
from archemist.core.models.lot_model import LotModel
from archemist.core.state.batch import Batch
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import LotStatus


class Lot:
    def __init__(self, lot_model: Union[LotModel, ModelProxy]) -> None:
        if isinstance(lot_model, ModelProxy):
            self._model_proxy = lot_model
        else:
            self._model_proxy = ModelProxy(lot_model)

    @classmethod
    def from_args(cls, batches: List[Batch]):
        model = LotModel()
        model.batches = [batch.model for batch in batches]
        model.save()
        # update batches lot parent id
        for batch in batches:
            batch.parent_lot_id = model.id
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = LotModel.objects.get(id=object_id)
        return cls(model)

    @property
    def model(self) -> LotModel:
        return self._model_proxy.model

    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

    @property
    def status(self) -> LotStatus:
        return self._model_proxy.status

    @status.setter
    def status(self, new_status: LotStatus):
        self._model_proxy.status = new_status

    @property
    def batches(self) -> List[Batch]:
        return ListProxy(self._model_proxy.batches, Batch)

    @property
    def num_batches(self) -> int:
        return len(self._model_proxy.batches)

    def add_station_stamp(self, station_stamp: str):
        for batch in self.batches:
            batch.add_station_stamp(station_stamp)

    @property
    def recipe(self) -> Recipe:
        if self.is_recipe_attached():
            return Recipe(self._model_proxy.recipe)

    def is_recipe_attached(self) -> bool:
        return self._model_proxy.recipe is not None

    def attach_recipe(self, recipe: Recipe):
        self._model_proxy.recipe = recipe.model

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.object_id}'

    def __eq__(self, __value: object) -> bool:
        return self.object_id == __value.object_id
