from bson.objectid import ObjectId
from multipledispatch import dispatch
from archemist.util import Location
from typing import List
from archemist.models.station_model import StationModel
from archemist.state.station import Station
from archemist.models.robot_model import RobotModel
from archemist.state.robot import Robot
from archemist.models.material_model import LiquidMaterialModel,SolidMaterialModel
from archemist.state.material import Liquid, Solid
from archemist.models.batch_model import BatchModel
from archemist.state.batch import Batch
from archemist.models.recipe_model import RecipeModel
from archemist.persistence.object_factory import RobotFactory,StationFactory
import importlib
import pkgutil


class State:
    def __init__(self):
        # to load all the derived modes for the station and robot queries to work
        pkg = importlib.import_module('archemist.state.stations')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            importlib.import_module(module_itr.name)

        pkg = importlib.import_module('archemist.state.robots')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            importlib.import_module(module_itr.name)
    
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

    def add_clean_batch(self, batch_id: int, num_samples: int, location:Location) -> Batch:
        return Batch.from_arguments(batch_id=batch_id, num_samples=num_samples, location=location)

    def get_completed_batches(self) -> List[Batch]:
        complete_recipes = RecipeModel.objects(current_state='end')
        return [Batch(model) for model in BatchModel.objects(recipe__in=complete_recipes)]

    def get_clean_batches(self) -> List[Batch]:
        return [Batch(model) for model in BatchModel.objects(recipe=None)]

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
        
        

