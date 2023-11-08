from __future__ import annotations
from typing import Any, Dict, Type, TYPE_CHECKING
if TYPE_CHECKING:
    from archemist.core.state.material import Liquid, Solid
    from archemist.core.state.robot import Robot, RobotModel
    from archemist.core.state.station import Station, StationModel
    from archemist.core.state.station_process import StationProcessModel, StationProcess
    from archemist.core.state.station_op import StationOp, StationOpModel
    from archemist.core.state.station_op_result import StationOpResult, StationOpResultModel
    from archemist.core.state.robot_op import RobotOp, RobotOpModel
    from archemist.core.state.lot import Lot

import importlib
import pkgutil
from bson.objectid import ObjectId


def _import_class_from_module(cls_name: str, module_path: str) -> Any:
    try:
        module = importlib.import_module(module_path)
        return getattr(module, cls_name, None)
    except (ModuleNotFoundError, ImportError):
        return None


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
    def create_op_handler(robot: Robot, use_sim_handler: bool=False):
        if use_sim_handler or robot.selected_handler == "SimRobotOpHandler":
            cls = _import_class_from_module('SimRobotOpHandler', 'archemist.core.processing.handler')
        else:
            handler_type =  robot.selected_handler
            robot_module_path = robot.module_path
            handler_module_path = robot_module_path.rsplit('.',1)[0] + '.handler'
            cls = _import_class_from_module(handler_type, handler_module_path)
        
        if cls:
                return cls(robot)
        
        raise NameError(f"Robot op handler type {robot.selected_handler} is not defined or have errors")
        
    
class RobotOpFactory:
    @staticmethod
    def create_from_args(op_type: str, op_params: Dict[str, Any] = None) -> Type[RobotOp]:
        if op_type in ["RobotOp", "RobotTaskOp", "RobotMaintenanceOp", \
            "RobotNavOp", "RobotWaitOp", "DropBatchOp", \
            "CollectBatchOp"]:
            
            cls = _import_class_from_module(op_type, 'archemist.core.state.robot_op')
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
    def create_from_model(op_model: Type[RobotOpModel]) -> Type[RobotOp]:
        cls = _import_class_from_module(op_model._type, op_model._module)
        return cls(op_model)

class StationFactory:
    @staticmethod
    def create_from_dict(station_dict: Dict) -> Type[Station]:
        if station_dict['type'] == "Station":
            cls = _import_class_from_module('Station', 'archemist.core.state.station')
            return cls.from_dict(station_dict)
        else:
            pkg = importlib.import_module('archemist.stations')
            for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                state_module = f'{module_itr.name}.state'
                cls = _import_class_from_module(station_dict['type'], state_module)
                if cls:
                    return cls.from_dict(station_dict)
       
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
    def create_op_handler(station: Station, use_sim_handler: bool=False):
        if use_sim_handler or station.selected_handler == "SimStationOpHandler":
            cls = _import_class_from_module('SimStationOpHandler', 'archemist.core.processing.handler')
        else:
            handler_type =  station.selected_handler
            station_module_path = station.module_path
            handler_module_path = station_module_path.rsplit('.',1)[0] + '.handler'
            cls = _import_class_from_module( handler_type,  handler_module_path)

        if cls:
            return cls(station)

        raise NameError(f"station op handler type {station.selected_handler} is not defined or have errors")
    
class StationOpFactory:
    @staticmethod
    def create_from_args(op_type: str, op_params: Dict[str, Any] = None) -> Type[StationOp]:
        if op_type == "StationOp":
            cls = _import_class_from_module('StationOp', 'archemist.core.state.station_op')
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
    def create_from_model(op_model: Type[StationOpModel]) -> Type[StationOp]:
        cls = _import_class_from_module(op_model._type, op_model._module)
        return cls(op_model)
    
    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[StationOp]:
        from archemist.core.state.station_op import StationOpModel
        model = StationOpModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)

class OpResultFactory:
    @staticmethod
    def create_from_model(result_model: Type[StationOpResultModel]) -> Type[StationOpResult]:
        cls = _import_class_from_module(result_model._type, result_model._module)
        return cls(result_model)
    
    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[StationOpResult]:
        from archemist.core.state.station_op_result import StationOpResultModel
        model = StationOpResultModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)
    
class ProcessFactory:
    @staticmethod
    def create_from_model(process_model: Type[StationProcessModel]) -> Type[StationProcess]:
        cls = _import_class_from_module(process_model._type, process_model._module)
        return cls(process_model)
    
    @staticmethod
    def create_from_dict(proc_dict: Dict, lot: Lot, station_module: str = None) -> Type[StationProcess]:
        proc_type = proc_dict["type"]
        operations = proc_dict.get("operations")
        args_dict = {} if not proc_dict.get("args") else proc_dict.get("args")
        if proc_type == "StationProcess":
            cls = _import_class_from_module(proc_type, 'archemist.core.state.station_process')
        else:
            if station_module:
                if station_module.endswith(".state"):
                    station_module  = station_module[:-6] + ".process"
                cls = _import_class_from_module(proc_type, station_module)
            else:
                found = False
                # first search all the station processes
                pkg = importlib.import_module('archemist.stations')
                for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                    process_module = f'{module_itr.name}.process'
                    cls = _import_class_from_module(proc_type, process_module)
                    if cls: 
                        found = True
                        break
                
                # if not found search processes module
                if not found:
                    pkg = importlib.import_module('archemist.processes')
                    for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
                        cls = _import_class_from_module(proc_type, module_itr.name)
                        if cls:
                            break

        if cls:
            return cls.from_args(lot=lot, operations=operations, **args_dict)
        else:
            raise NameError(f"StationProcess type {proc_type} is not defined")
        
    @staticmethod
    def create_from_object_id(object_id: ObjectId) -> Type[StationProcess]:
        from archemist.core.models.station_process_model import StationProcessModel
        model = StationProcessModel.objects.get(id=object_id)
        cls = _import_class_from_module(model._type, model._module)
        return cls(model)