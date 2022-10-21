from archemist.util.location import Location
from enum import Enum
from archemist.exceptions.exception import RobotAssignedRackError
from datetime import datetime
from mongoengine import Document, EmbeddedDocument, fields
from bson.objectid import ObjectId
from typing import Dict, List, Any
from archemist.persistence.object_factory import RobotFactory

class RobotState(Enum):
    OP_ASSIGNED = 0
    EXECUTING_OP = 1
    EXECUTION_COMPLETE = 2
    IDLE = 3

class RobotTaskType(Enum):
    LOAD_TO_ROBOT = 0
    UNLOAD_FROM_ROBOT = 1
    MANIPULATION = 2
    OTHER = 3

class RobotOpDescriptorModel(EmbeddedDocument):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    origin_station = fields.ObjectIdField(null=True)
    related_batch_id = fields.IntField(null=True)

    has_result = fields.BooleanField(default=False)
    was_successful = fields.BooleanField(default=False)
    robot_stamp = fields.StringField()
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'allow_inheritance':True}

class RobotOpDescriptor:
    def __init__(self, op_model: RobotOpDescriptorModel) -> None:
        self._model = op_model

    @property
    def model(self) -> RobotOpDescriptorModel:
        return self._model

    @property
    def origin_station(self) -> ObjectId:
        return self._model.origin_station

    @origin_station.setter
    def origin_station(self, station_object_id: ObjectId) -> None:
        self._model.origin_station = station_object_id

    @property
    def related_batch_id(self) -> int:
        return self._model.related_batch_id

    @related_batch_id.setter
    def related_batch_id(self, batch_id: int) -> None:
        self._model.related_batch_id = batch_id

    @property
    def has_result(self) -> bool:
        return self._model.has_result

    @property
    def was_successful(self) -> bool:
        return self._model.was_successful

    @property
    def start_timestamp(self) -> datetime:
        return self._model.start_timestamp

    @property
    def end_timestamp(self) -> datetime:
        return self._model.end_timestamp

    @property
    def robot_stamp(self):
        self._model.robot_stamp

    def add_start_timestamp(self):
        self._model.start_timestamp = datetime.now()

    def complete_op(self, robot_stamp: str, success: bool):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.robot_stamp = robot_stamp
        self._model.end_timestamp = datetime.now()

class RobotTaskOpDescriptorModel(RobotOpDescriptorModel):
    name = fields.StringField(required=True)
    task_type = fields.EnumField(RobotTaskType, required=True)
    params = fields.ListField(fields.StringField(), default=True)
    location = fields.DictField()

class RobotTaskOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, name: str, type: RobotTaskType=RobotTaskType.MANIPULATION, params: List[str]=[], 
                    location: Location=Location(), origin_station: ObjectId=None, related_batch_id: int=None):
        model = RobotTaskOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.name = name
        model.task_type = type
        model.params = [str(param) for param in params]
        model.location = location.to_dict() if location is not None else None
        model.origin_station = origin_station if origin_station is not None else None
        model.related_batch_id = related_batch_id if related_batch_id is not None else None
        return cls(model)
    
    @property
    def name(self):
        return self._model.name

    @property
    def task_type(self):
        return self._model.task_type

    @property
    def params(self):
        return self._model.params

    @property
    def location(self):
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self.name}, params: {self.params} @{self.location.get_map_coordinates()}'

class RobotModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    operational = fields.BooleanField(default=True)
    batch_capacity = fields.IntField(min_value=1, default=1)
    location = fields.DictField()
    assigned_op = fields.EmbeddedDocumentField(RobotOpDescriptorModel, null=True)
    complete_op = fields.EmbeddedDocumentField(RobotOpDescriptorModel, null=True)
    state = fields.EnumField(RobotState, default=RobotState.IDLE)
    robot_op_history = fields.EmbeddedDocumentListField(RobotOpDescriptorModel, default=[])

    meta = {'collection': 'robots', 'db_alias': 'archemist_state', 'allow_inheritance': True}


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

class MobileRobotModel(RobotModel):
    onboard_batches = fields.ListField(fields.IntField(),default=[])

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