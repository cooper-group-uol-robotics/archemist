from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy
from archemist.core.models.robot_model import RobotModel, MobileRobotModel, MobileRobotMode
from archemist.core.state.robot_op import RobotOpDescriptor, RobotTaskOpDescriptor
from archemist.core.util.enums import RobotState,RobotTaskType, OpState, OpResult
from archemist.core.util.location import Location
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from typing import Dict, List, Any, Union, Type
from archemist.core.persistence.object_factory import RobotOpFactory
from bson.objectid import ObjectId

class Robot:
    def __init__(self, robot_model: Union[RobotModel, ModelProxy]) -> None:
        if isinstance(robot_model, ModelProxy):
            self._model_proxy = robot_model
        else:
            self._model_proxy = ModelProxy(robot_model)
        

    @classmethod
    def from_dict(cls, robot_dict: Dict):
        model = RobotModel()
        cls._set_model_common_fields(model, robot_dict)
        model.save()
        return cls(model)

    @classmethod
    def _set_model_common_fields(cls, robot_model: RobotModel, robot_dict: Dict):
        robot_model._type = robot_dict['type']
        robot_model._module = cls.__module__
        robot_model.exp_id = robot_dict['id']
        robot_model.selected_handler = robot_dict['handler']
        if 'location' in robot_dict:
            robot_model.location = robot_dict['location']

    @property
    def model(self) -> RobotModel:
        return self._model_proxy.model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

    @property
    def id(self) -> int:
        return self._model_proxy.exp_id
    
    @property
    def module_path(self) -> str:
        return self._model_proxy._module

    @property
    def selected_handler(self) -> str:
       return self._model_proxy.selected_handler

    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        if loc_dict:
            return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, new_location: Location):
        if isinstance(new_location, Location):
            loc_dict = {'node_id':new_location.node_id, 'graph_id':new_location.graph_id, 'frame_name':new_location.frame_name}
            self._model_proxy.location = loc_dict
        else:
            raise ValueError

    @property
    def state(self) -> RobotState:
        return self._model_proxy.state
    
    @state.setter
    def state(self, new_state: RobotState):
        self._model_proxy.state = new_state

    @property
    def attending_to(self) -> ObjectId:
        return self._model_proxy.attending_to
    
    @attending_to.setter
    def attending_to(self, new_station: ObjectId):
        self._model_proxy.attending_to = new_station

    @property
    def queued_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.queued_ops, RobotOpFactory.create_from_model)

    @property
    def ops_history(self) -> List[Any]:
        return ListProxy(self._model_proxy.ops_history, RobotOpFactory.create_from_model)

    @property
    def assigned_op_state(self) -> OpState:
        return self._model_proxy.assigned_op_state

    @property
    def assigned_op(self) -> Type[RobotOpDescriptor]:
        return RobotOpFactory.create_from_model(self._model_proxy.assigned_op) \
               if self._model_proxy.assigned_op else None
    
    def update_assigned_op(self):
        if self.queued_ops and self.assigned_op is None:
            op = self.queued_ops.pop()
            self._model_proxy.assigned_op = op.model
            self._model_proxy.assigned_op_state = OpState.ASSIGNED
            if self.attending_to != op.requested_by:
                self.attending_to = op.requested_by

    def add_op(self, robot_op: Type[RobotOpDescriptor]):
        self.queued_ops.append(robot_op)
        self._log_robot(f'({robot_op}) is queued')
        
    def set_assigned_op_to_execute(self):
        self.assigned_op.add_start_timestamp()
        self._model_proxy.assigned_op_state = OpState.EXECUTING

    def complete_assigned_op(self, result: OpResult, clear_assigned_op: bool=True):
        op = self.assigned_op
        if op:
            op.complete_op(self._model_proxy.object_id, result)
            self._log_robot(f'{op} is complete')
            if clear_assigned_op:
                self.clear_assigned_op()
    
    def clear_assigned_op(self):
        op = self.assigned_op
        self._model_proxy.assigned_op = None
        self._model_proxy.assigned_op_state = OpState.INVALID
        self.ops_history.append(op)
        
        if not self.queued_ops:
                self.attending_to = None

    def repeat_assigned_op(self):
        if self.assigned_op:
            self._model_proxy.assigned_op_state = OpState.TO_BE_REPEATED
        else:
            self._log_robot('Unable to repeat. No op assigned')

    def skip_assigned_op(self):
        if self.assigned_op:
            self._model_proxy.assigned_op_state = OpState.TO_BE_SKIPPED
        else:
            self._log_robot('Unable to skip. No op assigned')

    def _log_robot(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'

class FixedRobot(Robot):
    def __init__(self, robot_model: Union[RobotModel, ModelProxy]) -> None:
        super().__init__(robot_model)

class MobileRobot(Robot):
    def __init__(self, robot_model: Union[MobileRobotModel, ModelProxy]) -> None:
       super().__init__(robot_model)
    
    @classmethod
    def from_dict(cls, robot_dict: Dict):
        model = MobileRobotModel()
        cls._set_model_common_fields(model, robot_dict)
        model.total_lot_capacity = robot_dict["total_lot_capacity"]
        model.onboard_capacity = robot_dict["onboard_capacity"]
        model.save()
        return cls(model)

    @property
    def total_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity
    
    @property
    def free_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity - len(self.consigned_lots)
    
    @property 
    def consigned_lots(self) -> List[Lot]:
        return ListProxy(self._model_proxy.consigned_lots, Lot)
    
    @property
    def onboard_capacity(self) -> int:
        return self._model_proxy.onboard_capacity
    
    @property
    def free_batch_capacity(self) -> int:
        return self.onboard_capacity - len(self.onboard_batches)

    @property 
    def onboard_batches(self) -> List[Batch]:
        return ListProxy(self._model_proxy.onboard_batches, Batch)
    
    @property
    def operational_mode(self) -> MobileRobotMode:
        return self._model_proxy.operational_mode
    
    @operational_mode.setter
    def operational_mode(self, new_mode: MobileRobotMode):
        self._model_proxy.operational_mode = new_mode

    def add_op(self, robot_op: type[RobotOpDescriptor]):
        if isinstance(robot_op, RobotTaskOpDescriptor) and robot_op.task_type == RobotTaskType.LOAD_TO_ROBOT:
            if robot_op.related_lot not in self.consigned_lots:
                self.consigned_lots.append(robot_op.related_lot)
            else:
                self._log_robot(f"{robot_op} cannot be added to the robot queue since robot has no free lot capacity")
        return super().add_op(robot_op)

    def complete_assigned_op(self, result: OpResult, clear_assigned_op: bool=True):
        op = self.assigned_op
        if op:
            if isinstance(op, RobotTaskOpDescriptor):
                if op.task_type == RobotTaskType.LOAD_TO_ROBOT:
                    self.onboard_batches.append(op.related_batch)
                elif op.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                    self.onboard_batches.remove(op.related_batch)
                    if all(batch not in self.onboard_batches for batch in op.related_lot.batches):
                        self.consigned_lots.remove(op.related_lot)
            super().complete_assigned_op(result, clear_assigned_op)