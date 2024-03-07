from archemist.core.persistence.object_factory import RobotFactory, StationFactory
from archemist.core.state.material import Liquid, LiquidMaterialModel, Solid, SolidMaterialModel
from archemist.core.state.station import Station, StationModel
from archemist.core.state.robot import Robot, RobotModel
from archemist.core.state.lot import Lot, LotModel
from archemist.core.util.enums import LotStatus
from archemist.core.state.batch import Batch, BatchModel
from archemist.core.state.recipe import Recipe, RecipeModel
from archemist.core.state.state import (InputState,
                                        InputStateModel,
                                        WorkflowState,
                                        WorkflowStateModel,
                                        OutputState,
                                        OutputStateModel)
from archemist.stations import import_stations_models
from archemist.robots import import_robots_models

from typing import List, Type
from multipledispatch import dispatch, Dispatcher
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
    get_station = Dispatcher("get_station")

    @staticmethod
    def get_stations(station_type: str = None) -> List[Type[Station]]:
        stations_list = []
        if station_type:
            stations_list = [StationFactory.create_from_model(
                model) for model in StationModel.objects(_type=station_type)]
        else:
            stations_list = [StationFactory.create_from_model(
                model) for model in StationModel.objects]

        return stations_list

    @get_station.register(ObjectId)
    def get_station(object_id: ObjectId) -> Type[Station]:
        return StationFactory.create_from_object_id(object_id)

    @get_station.register(str)
    def get_station(station_type: str) -> Type[Station]:
        model = StationModel.objects(_type=station_type).first()
        return StationFactory.create_from_model(model)

    @get_station.register(int, str)
    def get_station(station_id: int, station_type: str) -> Type[Station]:
        model = StationModel.objects.get(_type=station_type, exp_id=station_id)
        if model is not None:
            return StationFactory.create_from_model(model)


class RobotsGetter:
    # this is needed to query derived robots models
    import_robots_models()
    get_robot = Dispatcher("get_robot")

    @staticmethod
    def get_robots(robot_type: str = None) -> List[Type[Robot]]:
        robots_list = []
        if robot_type:
            robots_list = [RobotFactory.create_from_model(
                model) for model in RobotModel.objects(_type=robot_type)]
        else:
            robots_list = [RobotFactory.create_from_model(
                model) for model in RobotModel.objects]

        return robots_list

    @get_robot.register(ObjectId)
    def get_robot_objectId(object_id: ObjectId) -> Type[Robot]:
        return RobotFactory.create_from_object_id(object_id)

    @get_robot.register(str)
    def get_robot_str(robot_type: str) -> Type[Robot]:
        model = RobotModel.objects(_type=robot_type).first()
        return RobotFactory.create_from_model(model)

    @get_robot.register(int, str)
    def get_robot_int_str(robot_id: int, robot_type: str) -> Type[Robot]:
        model = RobotModel.objects.get(_type=robot_type, exp_id=robot_id)
        if model is not None:
            return RobotFactory.create_from_model(model)


class LotsGetter:
    @staticmethod
    def get_lots() -> List[Lot]:
        return [Lot(model) for model in LotModel.objects]

    @staticmethod
    def get_finished_lots() -> List[Lot]:
        return [Lot(model) for model in LotModel.objects(status=LotStatus.FINISHED)]

    @staticmethod
    def get_containing_lot(batch: Batch):
        model = LotModel.objects(batches=batch.model).first()
        if model is not None:
            return Lot(model)


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


class StateGetter:
    @staticmethod
    def get_input_state() -> InputState:
        model = InputStateModel.objects(_type="InputStateModel").first()
        return InputState(model)

    @staticmethod
    def get_workflow_state() -> WorkflowState:
        model = WorkflowStateModel.objects(_type="WorkflowStateModel").first()
        return WorkflowState(model)

    @staticmethod
    def get_output_state() -> OutputState:
        model = OutputStateModel.objects(_type="OutputStateModel").first()
        return OutputState(model)
