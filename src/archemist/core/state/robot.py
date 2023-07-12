from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.models.robot_model import RobotModel, MobileRobotModel, MobileRobotMode
from archemist.core.state.robot_op import RobotOpDescriptor, RobotTaskOpDescriptor
from archemist.core.util.enums import RobotState,RobotTaskType
from archemist.core.util.location import Location
from archemist.core.state.batch import Batch
from archemist.core.exceptions.exception import RobotAssignedRackError
from typing import Dict, List, Any, Union
from archemist.core.persistence.object_factory import RobotFactory

class Robot:
    def __init__(self, robot_model: Union[RobotModel, ModelProxy]) -> None:
        if isinstance(robot_model, ModelProxy):
            self._model_proxy = robot_model
        else:
            self._model_proxy = ModelProxy(robot_model)
        

    @classmethod
    def from_dict(cls, robot_dict: Dict):
        model = RobotModel()
        cls._set_model_common_fields(robot_dict, model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @staticmethod
    def _set_model_common_fields(robot_dict: Dict, robot_model: RobotModel):
        robot_model._type = robot_dict['type']
        robot_model.exp_id = robot_dict['id']
        robot_model.selected_handler = robot_dict['handler']
        if 'location' in robot_dict:
            robot_model.location = robot_dict['location']
        if 'batch_capacity' in robot_dict:
            robot_model.batch_capacity = robot_dict['batch_capacity']

    @property
    def model(self) -> RobotModel:
        return self._model_proxy.model

    @property
    def id(self) -> int:
        return self._model_proxy.exp_id

    @property
    def selected_handler_dict(self) -> str:
        robot_module = self._model_proxy._module.rsplit('.',1)[0]
        return {'type':self._model_proxy.selected_handler, 'module':robot_module}

    @property
    def operational(self) -> bool:
        return self._model_proxy.operational

    @operational.setter
    def operational(self, new_state: bool):
        self._model_proxy.operational = new_state

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

    @property
    def op_history(self) -> List[Any]:
        return ListProxy(self._model_proxy.op_history, RobotFactory.create_op_from_model)

    def get_assigned_op(self) -> RobotOpDescriptor:
        op_model = self._model_proxy.assigned_op
        if op_model:
            return RobotFactory.create_op_from_model(op_model)

    def has_assigned_op(self) -> bool:
        return self._model_proxy.assigned_op is not None

    def assign_op(self, robot_op: RobotOpDescriptor):
        if not self.has_assigned_op():
            self._model_proxy.assigned_op = robot_op.model
            self._log_robot(f'Job ({robot_op}) is assigned.')
            self._update_state(RobotState.OP_ASSIGNED)

        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def start_executing_op(self):
        op = self.get_assigned_op()
        op.add_start_timestamp()
        self._update_state(RobotState.EXECUTING_OP)

    def complete_assigned_op(self, success: bool):
        op = self.get_assigned_op()
        op.complete_op(self._model_proxy.object_id, success)
        
        self._model_proxy.assigned_op = None
        self._model_proxy.complete_op = op.model
        self.op_history.append(op)
        self._log_robot(f'Job ({op} is complete.')
        self._update_state(RobotState.EXECUTION_COMPLETE)

    def is_assigned_op_complete(self) -> bool:
        return self._model_proxy.complete_op is not None

    def get_complete_op(self) -> RobotOpDescriptor:
        if self.is_assigned_op_complete():
            complete_op = RobotFactory.create_op_from_model(self._model_proxy.complete_op)
            self._model_proxy.complete_op = None
            self._log_robot(f'Job ({complete_op}) is retrieved.')
            self._update_state(RobotState.IDLE)
            return complete_op

    def repeat_assigned_op(self):
        if self.has_assigned_op():
            self._update_state(RobotState.REPEAT_OP)
        else:
            self._log_robot('Unable to repeat. No op assigned')

    def skip_assigned_op(self):
        if self.has_assigned_op():
            self._update_state(RobotState.SKIP_OP)
        else:
            self._log_robot('Unable to skip. No op assigned')

    def _log_robot(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: RobotState):
        self._model_proxy.state = new_state
        self._log_robot(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'

class MobileRobot(Robot):
    def __init__(self, robot_model: Union[MobileRobotModel, ModelProxy]) -> None:
        if isinstance(robot_model, ModelProxy):
            self._model_proxy = robot_model
        else:
            self._model_proxy = ModelProxy(robot_model)
    
    @classmethod
    def from_dict(cls, robot_dict: Dict):
        model = MobileRobotModel()
        cls._set_model_common_fields(robot_dict, model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def batch_capacity(self) -> int:
        return self._model_proxy.batch_capacity

    @property 
    def onboard_batches(self) -> List[Batch]:
        return ListProxy(self._model_proxy.onboard_batches, Batch)
    
    @property
    def operational_mode(self) -> MobileRobotMode:
        return self._model_proxy.operational_mode
    
    @operational_mode.setter
    def operational_mode(self, new_mode: MobileRobotMode):
        self._model_proxy.operational_mode = new_mode

    def is_onboard_capacity_full(self) -> bool:
        return len(self.onboard_batches) == self.batch_capacity

    def is_batch_onboard(self, batch: Batch) -> bool:
        return batch in self.onboard_batches

    def get_complete_op(self) -> Any:
        complete_op = super().get_complete_op()
        if isinstance(complete_op, RobotTaskOpDescriptor):
            if complete_op.task_type == RobotTaskType.LOAD_TO_ROBOT:
                self._add_to_onboard_batches(complete_op.related_batch)
            elif complete_op.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                self._remove_from_onboard_batches(complete_op.related_batch)
        return complete_op
    
    def _add_to_onboard_batches(self, batch: Batch):
        if not self.is_onboard_capacity_full():
            self.onboard_batches.append(batch)
        else:
            self._log_robot(f'Cannot add batch {batch} to deck. Batch capacity exceeded')

    def _remove_from_onboard_batches(self, batch: Batch):
        self.onboard_batches.remove(batch)
