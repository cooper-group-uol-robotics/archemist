from bson.objectid import ObjectId
from multipledispatch import dispatch
from archemist.core.util import Location
from typing import Dict, List, Deque
from archemist.core.models.station_model import StationModel
from archemist.core.state.station import Station
from archemist.core.models.robot_model import RobotModel
from archemist.core.state.robot import Robot
from archemist.core.models.material_model import LiquidMaterialModel,SolidMaterialModel
from archemist.core.state.material import Liquid, Solid
from archemist.core.models.batch_model import BatchModel
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.models.recipe_model import RecipeModel
from archemist.core.persistence.object_factory import RobotFactory,StationFactory
from archemist.core.models.state_model import StateModel
from archemist.core.util.list_field_adapter import OpListAdapter, StateObjListAdapter
from collections import deque
import importlib
import pkgutil

class State:
    def __init__(self, state_model: StateModel):
        # to load all the derived modes for the station and robot queries to work
        pkg = importlib.import_module('archemist.stations')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            model_module = f'{module_itr.name}.model'
            importlib.import_module(model_module)

        pkg = importlib.import_module('archemist.robots')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            model_module = f'{module_itr.name}.model'
            importlib.import_module(model_module)
        self._model = state_model

    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = StateModel()
        model.workflow_name = state_dict['name']
        model.samples_per_batch = state_dict['samples_per_batch']
        model.default_batch_input_location = state_dict['default_batch_input_location']
        model.save()
        return cls(model)

    ''' Workflow objects list'''
    
    @property
    def liquids(self) -> List[Liquid]:
        return [Liquid(model) for model in LiquidMaterialModel.objects]

    @property
    def solids(self) -> List[Solid]:
        return [Solid(model) for model in SolidMaterialModel.objects]

    @property
    def stations(self) -> List[Station]:
        return [StationFactory.create_from_model(model) for model in StationModel.objects]

    @property
    def robots(self) -> List[Robot]:
        return[RobotFactory.create_from_model(model) for model in RobotModel.objects]

    @property
    def batches(self) -> List[Batch]:
        return [Batch(model) for model in BatchModel.objects]

    ''' Batch operations'''
    
    def add_clean_batch(self) -> Batch:
        batch_index = len(self.batches)
        samples_per_batch = self._model.samples_per_batch
        loc_dict = self._model.default_batch_input_location
        default_in_location = Location(loc_dict['node_id'],loc_dict['graph_id'])
        return Batch.from_arguments(batch_id=batch_index, num_samples=samples_per_batch, location=default_in_location)

    def get_completed_batches(self) -> List[Batch]:
        complete_recipes = RecipeModel.objects(current_state='end')
        return [Batch(model) for model in BatchModel.objects(recipe__in=complete_recipes)]

    def get_clean_batches(self) -> Deque[Batch]:
        return deque([Batch(model) for model in BatchModel.objects(recipe=None)])

    ''' Unassigned batches list operations '''

    @property
    def batches_buffer(self) -> StateObjListAdapter:
        return StateObjListAdapter(model=self._model, field_name='batches_buffer', state_obj_cls=Batch)

    ''' Robot queue operations '''
    @property
    def robot_ops_queue(self):
        return OpListAdapter(model=self._model, field_name='robot_ops_queue', factory_cls=RobotFactory)

    ''' Recipe operations'''
    def queue_recipe(self, recipe_dict: Dict):
        if not RecipeModel.objects(exp_id=recipe_dict['id']):
            recipe = Recipe.from_dict(recipe_dict)
            self._model.update(push__recipes_queue=recipe.model)
        else:
            print('Recipe not queued since it already exists in the database')

    @property
    def recipes_queue(self):
        return StateObjListAdapter(model=self._model, field_name='recipes_queue', state_obj_cls=Recipe)
        
    ''' Individual objects retreval operations'''
    
    @dispatch(ObjectId)
    def get_batch(self, object_id: ObjectId) -> Batch:
        model = BatchModel.objects.get(id=object_id)
        if model is not None:
            return Batch(model) 

    @dispatch(int)
    def get_batch(self, batch_id: int) -> Batch:
        model = BatchModel.objects.get(exp_id=batch_id)
        if model is not None:
            return Batch(model) 

    def is_batch_complete(self, batch_id: int) -> bool:
        model = BatchModel.objects.get(exp_id=batch_id)
        if model is not None:
            return model.recipe.current_state == 'end'

    @dispatch(ObjectId)
    def get_station(self, object_id: ObjectId) -> Station:
        return StationFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_station(self, station_type: str, station_id: int) -> Station:
        model = StationModel.objects.get(_type=station_type,exp_id=station_id)
        if model is not None:
            return StationFactory.create_from_model(model)

    @dispatch(ObjectId)
    def get_robot(self, object_id: ObjectId):
        return RobotFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_robot(self, robot_type: str, robot_id: int):
        model = RobotModel.objects.get(_type=robot_type,exp_id=robot_id)
        if model is not None:
            return RobotFactory.create_from_model(model)

    def get_robots(self, robot_type: str):
        return  [RobotFactory.create_from_model(model) for model in RobotModel.objects(_type=robot_type)]
        
        

