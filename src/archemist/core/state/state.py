from bson.objectid import ObjectId
from multipledispatch import dispatch
from typing import Dict, List, Union, Type

from archemist.core.state.station import Station, StationModel
from archemist.core.state.robot import Robot, RobotModel, RobotOpDescriptor
from archemist.core.state.material import Liquid, Solid, LiquidMaterialModel, SolidMaterialModel
from archemist.core.state.batch import Batch, BatchModel
from archemist.core.state.lot import Lot, LotModel
from archemist.core.state.recipe import Recipe, RecipeModel
from archemist.core.state.station_process import StationProcess
from archemist.core.persistence.object_factory import RobotFactory, RobotOpFactory,StationFactory, ProcessFactory
from archemist.core.models.state_model import StateModel
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.stations import import_stations_models
from archemist.robots import import_robots_models

# this is needed to query derived stations/robots models
import_stations_models()
import_robots_models()

class State:
    def __init__(self, state_model: Union[StateModel, ModelProxy]):
        if isinstance(state_model, ModelProxy):
            self._model_proxy = state_model
        else:
            self._model_proxy = ModelProxy(state_model)

    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = StateModel()
        model.workflow_name = state_dict['name']
        model.save()
        return cls(model)

    ''' Workflow objects list'''

    @property
    def workflow_name(self) -> str:
        return self._model_proxy.workflow_name
    
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

    ''' Unassigned lots '''

    @property
    def lots_buffer(self) -> List[Lot]:
        return ListProxy(self._model_proxy.lots_buffer, Lot)
    
    def get_completed_lots(self) -> List[Lot]:
        return [lot for lot in self.lots if lot.recipe.is_complete()]

    ''' Robot queue operations '''
    
    @property
    def robot_ops_queue(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.robot_ops_queue, RobotOpFactory.create_from_model)
    
    ''' station procs queue operations '''
    
    @property
    def proc_requests_queue(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.proc_requests_queue, ProcessFactory.create_from_model)

    ''' Recipe operations'''

    @property
    def recipes_queue(self) -> List[Recipe]:
        return ListProxy(self._model_proxy.recipes_queue, Recipe)
    
    def is_new_recipe_dict(self, recipe_dict: Dict) -> bool:
        return not RecipeModel.objects(exp_id=recipe_dict['general']['id'])
        
    ''' Individual objects retreval operations'''

    @dispatch(ObjectId)
    def get_station(self, object_id: ObjectId) -> Type[Station]:
        return StationFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_station(self, station_type: str, station_id: int) -> Type[Station]:
        model = StationModel.objects.get(_type=station_type,exp_id=station_id)
        if model is not None:
            return StationFactory.create_from_model(model)

    @dispatch(ObjectId)
    def get_robot(self, object_id: ObjectId) -> Type[Robot]:
        return RobotFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_robot(self, robot_type: str, robot_id: int) -> Type[Robot]:
        model = RobotModel.objects.get(_type=robot_type,exp_id=robot_id)
        if model is not None:
            return RobotFactory.create_from_model(model)

    def get_robots_of_type(self, robot_type: str) -> List[Type[Robot]]:
        return  [RobotFactory.create_from_model(model) for model in RobotModel.objects(_type=robot_type)]
        
        

