from archemist.core.persistence.object_factory import RobotFactory, StationFactory
from archemist.core.state.material import Liquid, LiquidMaterialModel, Solid, SolidMaterialModel
from archemist.core.state.station import Station, StationModel
from archemist.core.state.robot import Robot, RobotModel
from archemist.core.state.lot import Lot, LotModel
from archemist.core.util.enums import LotStatus
from archemist.core.state.batch import Batch, BatchModel
from archemist.core.state.recipe import Recipe, RecipeModel
from archemist.stations import import_stations_models
from archemist.robots import import_robots_models

from typing import List, Type
from multipledispatch import dispatch
from bson.objectid import ObjectId

class MaterialsGetter:
    @staticmethod
    def get_liquids() -> List[Liquid]:
        return [Liquid(model) for model in LiquidMaterialModel.objects]
    
    @staticmethod
    def get_solids() -> List[Solid]:
        return [Solid(model) for model in SolidMaterialModel.objects]
    
class StationsGetter:
    # this is needed to query derived stations models
    import_stations_models()
    
    @staticmethod
    def get_stations(station_type: str=None) -> List[Type[Station]]:
        stations_list = []
        if station_type:
            stations_list = [StationFactory.create_from_model(model) for model in StationModel.objects(_type=station_type)]
        else:
            stations_list = [StationFactory.create_from_model(model) for model in StationModel.objects]
        
        return stations_list
    
    @dispatch(ObjectId)
    def get_station(object_id: ObjectId) -> Type[Station]:
        return StationFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_station(station_type: str, station_id: int) -> Type[Station]:
        model = StationModel.objects.get(_type=station_type,exp_id=station_id)
        if model is not None:
            return StationFactory.create_from_model(model)
        
class RobotsGetter:
     # this is needed to query derived robots models
    import_robots_models()

    @staticmethod
    def get_robots(robot_type: str=None) -> List[Type[Robot]]:
        robots_list = []
        if robot_type:
            robots_list = [RobotFactory.create_from_model(model) for model in RobotModel.objects(_type=robot_type)]
        else:
            robots_list = [RobotFactory.create_from_model(model) for model in RobotModel.objects]
        
        return robots_list
    
    @dispatch(ObjectId)
    def get_robot(object_id: ObjectId) -> Type[Robot]:
        return RobotFactory.create_from_object_id(object_id)

    @dispatch(str, int)
    def get_robot(robot_type: str, robot_id: int) -> Type[Robot]:
        model = RobotModel.objects.get(_type=robot_type,exp_id=robot_id)
        if model is not None:
            return RobotFactory.create_from_model(model)
        
class LotsGetter:
    @staticmethod
    def get_lots() -> List[Lot]:
        return [Lot(model) for model in LotModel.objects]
    
    @staticmethod
    def get_finished_lots() -> List[Lot]:
        return [Lot(model) for model in LotModel.objects(status=LotStatus.FINISHED)]
    
class BatchesGetter:
    @staticmethod
    def get_batches() -> List[Batch]:
        return [Batch(model) for model in BatchModel.objects]
    
class RecipesGetter:
    @staticmethod
    def get_recipes() -> List[Recipe]:
        return [Recipe(model) for model in RecipeModel.objects]

    @staticmethod
    def recipe_exists(exp_id: int) -> bool:
        return bool(RecipeModel.objects(exp_id=exp_id))