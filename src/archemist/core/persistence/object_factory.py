from __future__ import annotations
from typing import Any, Dict, List, Type, TYPE_CHECKING
if TYPE_CHECKING:
    from archemist.core.state.material import Liquid, Solid
    from archemist.core.state.robot import Robot, RobotModel
    from archemist.core.state.station import Station, StationModel
    from archemist.core.state.station_process import StationProcessModel, StationProcess
    from archemist.core.state.station_op import StationOpDescriptor, StationOpDescriptorModel
    from archemist.core.state.robot_op import RobotOpDescriptor, RobotOpDescriptorModel
    from archemist.core.state.lot import Lot

import importlib
import pkgutil
from bson.objectid import ObjectId


def _import_class_from_module(cls_name: str, module_path: str) -> Any:
    module = importlib.import_module(module_path)
    return getattr(module, cls_name, None)


class RobotFactory:
    @staticmethod
    def create_from_dict(robot_dict: Dict) -> Type[Robot]:
        if robot_dict['type'] == "Robot" or robot_dict['type'] == "MobileRobot":
            cls = _import_class_from_module(robot_dict['type'], 'archemist.core.state.robot')
            return cls.from_dict(robot_dict)
        else:
            pkg = importlib.import_module('archemist.robots')
            for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                state_module = f'{module_itr.name}.state'
                cls = _import_class_from_module(robot_dict['type'], state_module)
                if cls:
                    return cls.from_dict(robot_dict)
        
        raise NameError(f"Robot type {robot_dict['type']} is not defined")

    @staticmethod
    def create_from_model(robot_model: RobotModel) -> Type[Robot]:
        cls = _import_class_from_module(robot_model._type, robot_model._module)
        return cls(robot_model)

    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[Robot]:
        from archemist.core.state.robot import RobotModel
        model = RobotModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)

    @staticmethod
    def create_handler(robot: Robot, use_sim_handler: bool=False):
        if use_sim_handler or robot.selected_handler == "GenericRobotHandler":
            cls = _import_class_from_module('GenericRobotHandler', 'archemist.robots.simulated_robot.handler')
        else:
            handler_type =  robot.selected_handler
            robot_module_path = robot.module_path
            handler_module_path = robot_module_path.rsplit('.',1)[0] + '.handler'
            cls = _import_class_from_module(handler_type, handler_module_path)
        return cls(robot)
    
class RobotOpFactory:
    @staticmethod
    def create_from_args(op_type: str, op_params: Dict[str, Any] = None) -> Type[RobotOpDescriptor]:
        if op_type == "RobotOpDescriptor":
            cls = _import_class_from_module('RobotOpDescriptor', 'archemist.core.state.robot_op')
            return cls.from_args()
        elif op_type == "RobotTaskOpDescriptor":
            cls = _import_class_from_module('RobotTaskOpDescriptor', 'archemist.core.state.robot_op')
            params = op_params if op_params is not None else {}
            return cls.from_args(**params)
        else:
            pkg = importlib.import_module('archemist.robots')
            for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                state_module = f'{module_itr.name}.state'
                cls = _import_class_from_module(op_type, state_module)
                if cls:
                    params = op_params if op_params is not None else {}
                    return cls.from_args(**params)
        
        
        raise NameError(f"Robot op type {op_type} is not defined")

    @staticmethod
    def create_from_model(op_model: Type[RobotOpDescriptorModel]) -> Type[RobotOpDescriptor]:
        cls = _import_class_from_module(op_model._type, op_model._module)
        return cls(op_model)

class StationFactory:
    @staticmethod
    def create_from_dict(station_dict: Dict, liquids: List[Liquid] = None, solids: List[Solid] = None) -> Type[Station]:
        if station_dict['type'] == "Station":
            cls = _import_class_from_module('Station', 'archemist.core.state.station')
            return cls.from_dict(station_dict, liquids, solids)
        else:
            pkg = importlib.import_module('archemist.stations')
            for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                state_module = f'{module_itr.name}.state'
                cls = _import_class_from_module(station_dict['type'], state_module)
                if cls:
                    return cls.from_dict(station_dict, liquids, solids)
       
        raise NameError(f"Station type {station_dict['type']} is not defined")

    @staticmethod
    def create_from_model(station_model: StationModel) -> Type[Station]:
        module = importlib.import_module(station_model._module)
        cls = getattr(module, station_model._type)
        cls = _import_class_from_module(station_model._type, station_model._module)
        return cls(station_model)

    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[Station]:
        from archemist.core.state.station import StationModel
        model = StationModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)

    @staticmethod
    def create_handler(station: Station, use_sim_handler: bool=False):
        if use_sim_handler or station.selected_handler == "GenericStationHandler":
            cls = _import_class_from_module('GenericStationHandler', 'archemist.stations.simulated_station.handler')
        else:
            handler_type =  station.selected_handler
            station_module_path = station.module_path
            handler_module_path = station_module_path.rsplit('.',1)[0] + '.handler'
            cls = _import_class_from_module( handler_type,  handler_module_path)
        return cls(station)
    
class StationOpFactory:
    @staticmethod
    def create_from_args(op_type: str, op_params: Dict[str, Any] = None) -> Type[StationOpDescriptor]:
        if op_type == "StationOpDescriptor":
            cls = _import_class_from_module('StationOpDescriptor', 'archemist.core.state.station_op')
            return cls.from_args()
        else:
            pkg = importlib.import_module('archemist.stations')
            for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                state_module = f'{module_itr.name}.state'
                cls = _import_class_from_module(op_type, state_module)
                if cls:
                    params = op_params if op_params is not None else {}
                    return cls.from_args(**params)
        
        
        raise NameError(f"Station op type {op_type} is not defined")


    @staticmethod
    def create_from_model(op_model: Type[StationOpDescriptorModel]) -> Type[StationOpDescriptor]:
        cls = _import_class_from_module(op_model._type, op_model._module)
        return cls(op_model)
    
    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[StationOpDescriptor]:
        from archemist.core.state.station_op import StationOpDescriptorModel
        model = StationOpDescriptorModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)
    
class ProcessFactory:
    @staticmethod
    def create_from_model(process_model: Type[StationProcessModel]) -> Type[StationProcess]:
        cls = _import_class_from_module(process_model._type, process_model._module)
        return cls(process_model)
    
    @staticmethod
    def create_from_args(proc_type: str, lot: Lot, key_process_ops: Dict[str, Type[StationOpDescriptor]],
                         processing_slot: int = None, args_dict: Dict = {},
                         station_module: str = None) -> Type[StationProcess]:
        if proc_type == "StationProcess":
            cls = _import_class_from_module(proc_type, 'archemist.core.state.station_process')
        else:
            if station_module:
                cls = _import_class_from_module(proc_type, station_module)
            else:
                pkg = importlib.import_module('archemist.stations')
                for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                    process_module = f'{module_itr.name}.process'
                    cls = _import_class_from_module(proc_type, process_module)
                    if cls: 
                        break

        if cls:
            return cls.from_args(lot, key_process_ops, processing_slot, args_dict)
        else:
            raise NameError(f"StationProcess type {proc_type} is not defined")
        
    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[StationProcess]:
        from archemist.core.models.station_process_model import StationProcessModel
        model = StationProcessModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)