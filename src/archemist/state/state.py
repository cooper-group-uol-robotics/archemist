from bson.objectid import ObjectId
from multipledispatch import dispatch
from archemist.util import Location
from typing import List
from archemist.state.station import StationModel, Station
from archemist.state.robot import RobotModel, Robot
from archemist.state.material import Liquid,LiquidMaterialModel,SolidMaterialModel, Solid
from archemist.state.batch import BatchModel,Batch
from archemist.state.recipe import RecipeModel
from archemist.persistence.object_factory import RobotFactory,StationFactory


class State:
    def __init__(self):
        pass
    
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
        
        

