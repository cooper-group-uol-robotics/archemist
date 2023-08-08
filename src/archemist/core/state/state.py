from bson.objectid import ObjectId
from multipledispatch import dispatch
from archemist.core.util import Location
from typing import Dict, List, Deque, Union, Type
from archemist.core.state.station import Station, StationModel
from archemist.core.state.robot import Robot, RobotModel, RobotOpDescriptor
from archemist.core.state.material import Liquid, Solid, LiquidMaterialModel, SolidMaterialModel
from archemist.core.state.batch import Batch, BatchModel
from archemist.core.state.lot import Lot, LotModel
from archemist.core.state.recipe import Recipe, RecipeModel
from archemist.core.persistence.object_factory import RobotFactory,StationFactory
from archemist.core.models.state_model import StateModel
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from collections import deque
import importlib
import pkgutil

class State:
    def __init__(self, state_model: Union[StateModel, ModelProxy]):
        # to load all the derived modes for the station and robot queries to work
        pkg = importlib.import_module('archemist.stations')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            model_module = f'{module_itr.name}.model'
            importlib.import_module(model_module)

        pkg = importlib.import_module('archemist.robots')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            model_module = f'{module_itr.name}.model'
            importlib.import_module(model_module)
        if isinstance(state_model, ModelProxy):
            self._model_proxy = state_model
        else:
            self._model_proxy = ModelProxy(state_model)

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
    def lots(self) -> List[Lot]:
        return [Lot(model) for model in LotModel.objects]
    
    @property
    def batches(self) -> List[Batch]:
        return [Batch(model) for model in BatchModel.objects]
    
    @property
    def recipes(self) -> List[Recipe]:
        return [Recipe(model) for model in RecipeModel.objects]

    ''' Batch operations'''
    
    def add_clean_batch(self) -> Batch:
        batch_index = len(self.batches)
        samples_per_batch = self._model_proxy.samples_per_batch
        loc_dict = self._model_proxy.default_batch_input_location
        default_in_location = Location(loc_dict['node_id'],loc_dict['graph_id'])
        return Batch.from_arguments(batch_id=batch_index, num_samples=samples_per_batch, location=default_in_location)

    def get_completed_batches(self) -> List[Batch]:
        complete_recipes = RecipeModel.objects(current_state='end_state')
        return [Batch(model) for model in BatchModel.objects(recipe__in=complete_recipes)]

    def get_clean_batches(self) -> Deque[Batch]:
        return deque([Batch(model) for model in BatchModel.objects(recipe=None)])

    ''' Unassigned batches list operations '''

    @property
    def batches_buffer(self) -> List[Batch]:
        return ListProxy(self._model_proxy.batches_buffer, Batch)

    ''' Robot queue operations '''
    
    @property
    def robot_ops_queue(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.robot_ops_queue, RobotFactory.create_op_from_model)

    ''' Recipe operations'''

    @property
    def recipes_queue(self) -> List[Recipe]:
        return ListProxy(self._model_proxy.recipes_queue, Recipe)
    
    def queue_recipe(self, recipe_dict: Dict):
        if not RecipeModel.objects(exp_id=recipe_dict['general']['id']):
            recipe = Recipe.from_dict(recipe_dict)
            self.recipes_queue.append(recipe)
        else:
            print('Recipe not queued since it already exists in the database')
        
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
            return model.recipe.current_state == 'end_state'

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
        
        

