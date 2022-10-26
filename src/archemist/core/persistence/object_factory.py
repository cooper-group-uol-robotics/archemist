from __future__ import annotations
from typing import Any, Dict, List, TYPE_CHECKING
if TYPE_CHECKING:
    from archemist.core.state.material import Material,Liquid, Solid
    from archemist.core.state.batch import Batch
    from archemist.core.state.robot import Robot, RobotOpDescriptor, RobotModel
    from archemist.core.state.station import Station, StationOpDescriptor, StationModel

import importlib
import pkgutil
from bson.objectid import ObjectId


class MaterialFactory:

    @staticmethod
    def create_liquid_from_dict( liquid_dict: Dict) -> Liquid:
        from archemist.core.state.material import Liquid
        return Liquid.from_dict(liquid_dict)

    @staticmethod
    def create_solid_from_dict(solid_dict: Dict) -> Solid:
        from archemist.core.state.material import Solid
        return Solid.from_dict(solid_dict)

    @staticmethod
    def create_material_from_object_id(object_id:ObjectId) -> Material:
        from archemist.core.state.material import MaterialModel,Liquid, Solid
        model = MaterialModel.objects.get(id=object_id)
        if model._type == 'Liquid':
            return Liquid(model)
        elif model._type == 'Solid':
            return Solid(model)

class RobotFactory:
    
    @staticmethod
    def create_from_dict(robot_dict: Dict) -> Robot:
        pkg = importlib.import_module('archemist.core.state.robots')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            module = importlib.import_module(module_itr.name)
            if hasattr(module,robot_dict['type']):
                cls = getattr(module,robot_dict['type'])
                return cls.from_dict(robot_dict)

    @staticmethod
    def create_from_model(robot_model: RobotModel) -> Robot:
        module = importlib.import_module(robot_model._module)
        cls = getattr(module, robot_model._type)
        return cls(robot_model)

    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Robot:
        from archemist.core.state.robot import RobotModel
        model = RobotModel.objects.get(id=object_id)
        module = importlib.import_module(model._module)
        cls = getattr(module,model._type)
        return cls(model)

    @staticmethod
    def create_op_from_model(op_model: Any) -> RobotOpDescriptor:
        module = importlib.import_module(op_model._module)
        cls = getattr(module,op_model._type)
        return cls(op_model)

class StationFactory:

    @staticmethod
    def create_from_dict(station_dict: Dict, liquids: List[Liquid], solids: List[Solid]) -> Station:
        pkg = importlib.import_module('archemist.core.state.stations')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            module = importlib.import_module(module_itr.name)
            if hasattr(module,station_dict['type']):
                cls = getattr(module,station_dict['type'])
                return cls.from_dict(station_dict, liquids, solids)

    @staticmethod
    def create_from_model(station_model: StationModel) -> Station:
        module = importlib.import_module(station_model._module)
        cls = getattr(module, station_model._type)
        return cls(station_model)

    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Station:
        from archemist.core.state.station import StationModel
        model = StationModel.objects.get(id=object_id)
        module = importlib.import_module(model._module)
        cls = getattr(module,model._type)
        return cls(model)


    @staticmethod
    def create_op_from_model(op_model: Any) -> StationOpDescriptor:
        module = importlib.import_module(op_model._module)
        cls = getattr(module,op_model._type)
        return cls(op_model)

    @staticmethod
    def create_op_from_dict(op_dict: Dict) -> StationOpDescriptor:
        pkg = importlib.import_module('archemist.core.state.stations')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            module = importlib.import_module(module_itr.name)
            if hasattr(module,op_dict['type']):
                cls = getattr(module,op_dict['type'])
                kwargs = {} if op_dict['properties'] is None else op_dict['properties']
                return cls.from_args(**kwargs)

    @staticmethod
    def create_state_machine(station: Station):
        station_sm_dict = station.process_sm_dict
        pkg = importlib.import_module('archemist.core.processing.state_machines')
        for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
            module = importlib.import_module(module_itr.name)
            if hasattr(module,station_sm_dict['type']):
                cls = getattr(module,station_sm_dict['type'])
                return cls(station, station_sm_dict['args'])