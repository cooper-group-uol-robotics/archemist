from archemist.models.robot_model import RobotModel, MobileRobotModel
from archemist.state.robot_op import RobotOpDescriptor, RobotTaskOpDescriptor
from archemist.util.enums import RobotState,RobotTaskType
from archemist.util.location import Location
from archemist.exceptions.exception import RobotAssignedRackError
from typing import Dict, List, Any
from archemist.persistence.object_factory import RobotFactory

class Robot:
    def __init__(self, robot_model: RobotModel) -> None:
        self._model = robot_model

    @classmethod
    def from_dict(cls, robot_dict: Dict):
        pass

    @staticmethod
    def _set_model_common_fields(robot_dict: Dict, robot_model: RobotModel):
        robot_model._type = robot_dict['type']
        robot_model.exp_id = robot_dict['id']
        if 'location' in robot_dict:
            robot_model.location = robot_dict['location']
        if 'batch_capacity' in robot_dict:
            robot_model.batch_capacity = robot_dict['batch_capacity']

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def operational(self) -> bool:
        self._model.reload('operational')
        return self._model.operational

    @operational.setter
    def operational(self, new_state: bool):
        self._model.update(operational=new_state)

    @property
    def batch_capacity(self):
        return self._model.batch_capacity

    @property
    def location(self) -> Location:
        self._model.reload('location')
        loc_dict = self._model.location
        if loc_dict is not None:
            return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, new_location: Location):
        if isinstance(new_location, Location):
            loc_dict = {'node_id':new_location.node_id, 'graph_id':new_location.graph_id, 'frame_name':new_location.frame_name}
            self._model.update(location=loc_dict)
        else:
            raise ValueError

    @property
    def state(self) -> RobotState:
        self._model.reload('state')
        return self._model.state

    @property
    def robot_op_history(self) -> List[Any]:
        self._model.reload('robot_op_history')
        return [RobotFactory.create_op_from_model(op) for op in self._model.robot_op_history] 

    def get_assigned_op(self) -> RobotOpDescriptor:
        self._model.reload('assigned_op')
        op_model = self._model.assigned_op
        if op_model is not None:
            return RobotFactory.create_op_from_model(op_model)

    def has_assigned_op(self) -> bool:
        self._model.reload('assigned_op')
        return self._model.assigned_op is not None

    def assign_op(self, robot_op: RobotOpDescriptor):
        if not self.has_assigned_op():
            self._model.update(assigned_op=robot_op.model)
            self._log_robot(f'Job ({robot_op}) is assigned.')
            self._update_state(RobotState.OP_ASSIGNED)

        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def start_executing_op(self):
        op = self.get_assigned_op()
        op.add_start_timestamp()
        self._model.update(assigned_op=op.model)
        self._update_state(RobotState.EXECUTING_OP)

    def set_to_execution_complete(self):
        self._update_state(RobotState.EXECUTION_COMPLETE)

    def complete_assigned_op(self, success: bool):
        robot_stamp = f'{self._model._type}-{self.id}'
        job = self.get_assigned_op()
        job.complete_op(robot_stamp, success)
        self._model.update(complete_op=job.model)
        self._model.update(push__robot_op_history=job.model)
        self._model.update(unset__assigned_op=True)
        complete_op = RobotFactory.create_op_from_model(job.model)
        self._log_robot(f'Job ({complete_op} is complete.')

    def is_assigned_op_complete(self) -> bool:
        self._model.reload('complete_op')
        return self._model.complete_op is not None

    def get_complete_op(self) -> RobotOpDescriptor:
        if self.is_assigned_op_complete():
            complete_op = RobotFactory.create_op_from_model(self._model.complete_op)
            self._model.update(unset__complete_op=True)
            self._log_robot(f'Job ({complete_op}) is retrieved.')
            self._update_state(RobotState.IDLE)
            return complete_op

    def _log_robot(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: RobotState):
        self._model.update(state=new_state)
        self._log_robot(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'

class MobileRobot(Robot):
    def __init__(self, robot_model: MobileRobotModel) -> None:
        self._model = robot_model

    @property
    def batch_capacity(self):
        return self._model.batch_capacity

    @property 
    def onboard_batches(self):
        self._model.reload('onboard_batches')
        return self._model.onboard_batches

    def add_to_onboard_batches(self, batch_id: int):
        if len(self.onboard_batches) < self.batch_capacity:
            self._model.update(push__onboard_batches=batch_id)
        else:
            self._log_robot(f'Cannot add batch {batch_id} to deck. Batch capacity exceeded')

    def remove_from_onboard_batches(self, batch_id:int):
        self._model.update(pull__onboard_batches=batch_id)

    def is_onboard_capacity_full(self):
        return len(self.onboard_batches) == self.batch_capacity

    def is_batch_onboard(self, batch_id: int):
        return batch_id in self.onboard_batches

    def get_complete_op(self) -> Any:
        if self.is_assigned_op_complete():
            complete_op = RobotFactory.create_op_from_model(self._model.complete_op)
            self._model.update(unset__complete_op=True)
            if isinstance(complete_op, RobotTaskOpDescriptor):
                if complete_op.task_type == RobotTaskType.LOAD_TO_ROBOT:
                    self.add_to_onboard_batches(complete_op.related_batch_id)
                elif complete_op.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                    self.remove_from_onboard_batches(complete_op.related_batch_id)
            self._log_robot(f'Job ({complete_op}) is retrieved.')
            self._update_state(RobotState.IDLE)
            return complete_op