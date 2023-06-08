from typing import Dict, List
from archemist.core.models.state_model import StateModel, BatchModel, RecipeModel
from model import BatchModel, RecipeModel
from bson.objectid import ObjectId
from archemist.core.util import Location

from bson.objectid import ObjectId
from archemist.core.persistence.object_factory import StationFactory
from transitions import Machine

class Recipe:
    def __init__(self, recipe_model: RecipeModel):
        self._model = recipe_model
        self._station_sm = Machine(states=self._model.states,initial=self._model.current_state, transitions=self._model.transitions)
        
    @classmethod
    def from_dict(cls, recipe_document: dict):
        model = RecipeModel()
        model.name = recipe_document['general']['name']
        model.exp_id = recipe_document['general']['id']
        model.states.append('end_state')
        model.current_state = model.states[0]
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = RecipeModel.objects.get(id=object_id)
        return cls(model)
        
    @property
    def name(self):
        return self._model.name

    @property
    def id(self):
        return self._model.exp_id

    @property
    def model(self):
        self._model.reload()
        return self._model

    @property
    def current_state(self):
        self._model.reload('current_state')
        return self._model.current_state

    def is_complete(self):
        return self.current_state == 'end_state'
    
    def set_end_state(self):
        self._model.update(current_state = 'end_state')

        
    def _logRecipe(self, message: str):
        print(f'Recipe [{self.id}]: ' + message)


class Batch:
    def __init__(self, batch_model: BatchModel) -> None:
        self._model = batch_model

    @classmethod
    def from_arguments(cls, batch_id: int, num_samples: int, location:Location):
        model = BatchModel()
        model.exp_id = batch_id
        model.num_samples = num_samples
        model.location = location.to_dict()
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = BatchModel.objects.get(id=object_id)
        return cls(model)

    @property
    def model(self) -> BatchModel:
        self._model.reload()
        return self._model

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def recipe_attached(self) -> bool:
        self._model.reload('recipe')
        return self._model.recipe is not None

    @property
    def recipe(self) -> Recipe:
        if self.recipe_attached:
            return Recipe(self._model.recipe)
    
    @property
    def result(self):
        return self._model.result

    def attach_recipe(self, recipe: Recipe):
        self._model.update(recipe=recipe.model)
    
    def add_result(self, result: Dict):
        self._model.update(result = result)

    @property
    def location(self) -> Location:
        self._model.reload('location')
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._model.update(location=location.to_dict())
        else:
            raise ValueError

    @property
    def num_samples(self) -> int:
        return self._model.num_samples
          

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'

