from email.policy import default
from bson.objectid import ObjectId
from mongoengine import Document, EmbeddedDocument, fields
from enum import Enum
from pickle import loads, dumps
from typing import List, Any
from datetime import datetime
from archemist.state.material import Liquid,Solid
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from archemist.state.robot import RobotOpDescriptor
from archemist.state.batch import Batch, BatchModel
from archemist.persistence.object_factory import ObjectFactory

class StationState(Enum):
    IDLE = 0
    PROCESSING = 1
    WAITING_ON_ROBOT = 2
    WAITING_ON_OPERATION = 3
    OPERATION_COMPLETE = 4
    PROCESSING_COMPLETE = 5

class StationOpDescriptorModel(EmbeddedDocument):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    has_result = fields.BooleanField(default=False)
    was_successful = fields.BooleanField(default=False)
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'allow_inheritance': True}

class StationOpDescriptor:
    def __init__(self, stationOpModel: StationOpDescriptorModel) -> None:
        self._model = stationOpModel

    @property
    def model(self):
        return self._model

    @property
    def has_result(self):
        return self._model.has_result

    @property
    def was_successful(self):
        return self._model.was_successful

    @property
    def start_timestamp(self):
        return self._model.start_timestamp

    @property
    def end_timestamp(self):
        return self._model.end_timestamp

    def add_start_timestamp(self):
        self._model.start_timestamp = datetime.now()

    def complete_op(self, success: bool):
        self._model.has_result = True
        self.was_successful = success
        self._model.end_timestamp = datetime.now()

class StationOutputDescriptor:
    def __init__(self):
        self._has_result = False
        self._success = False
        self._timestamp = None

    @property
    def success(self):
        return self._success

    @success.setter
    def success(self, value):
        if isinstance(value, bool):
            self._success = value
        else:
            raise ValueError

    @property
    def has_result(self):
        return self._has_result

    @has_result.setter
    def has_result(self, value):
        if isinstance(value, bool):
            self._has_result = value
        else:
            raise ValueError

    def add_timestamp(self):
        self._timestamp = datetime.now()

# class StationOpDescriptor:
#     def __init__(self, stationName: str, output: StationOutputDescriptor):
#         self._stationName = stationName
#         self._output = output
#         self._timestamp = None

#     @property
#     def stationName(self):
#         return self._stationName

#     @property
#     def output(self):
#         return self._output

#     def add_timestamp(self):
#         self._timestamp = datetime.now()


class StationModel(Document):
    _type = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    location = fields.DictField()
    batch_capacity = fields.IntField(min_value=1, default=1)
    process_state_machine = fields.DictField(required=True)
    operational = fields.BooleanField(default=True)
    state = fields.EnumField(StationState, default=StationState.IDLE)
    loaded_samples = fields.IntField(default=0) # cannot use min value, breaks dec__ operator
    assigned_batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    processed_batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    requested_robot_op = fields.BinaryField(null=True)
    current_station_op = fields.EmbeddedDocumentField(StationOpDescriptorModel,null=True)
    station_op_history = fields.EmbeddedDocumentListField(StationOpDescriptorModel,default=[])
    requested_robot_op_history = fields.ListField(fields.BinaryField(), default=[])

    meta = {'collection': 'stations', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class Station:
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_document: dict, liquids: List[Liquid], solids: List[Solid]):
        pass

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        pass

    @classmethod
    def _set_model_common_fields(cls, station_document: dict, station_model: StationModel):
        station_model.exp_id = station_document['id']
        station_model.location = station_document['location']
        station_model.batch_capacity = station_document['batch_capacity']
        station_model.process_state_machine = station_document['process_state_machine']

    @property
    def model(self) -> StationModel:
        self._model.reload()
        return self._model


    @property
    def state(self) -> StationState:
        self._model.reload('state')
        return self._model.state

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def operational(self) -> bool:
        self._model.reload('operational')
        return self.model.operational

    @operational.setter
    def operational(self, new_state: bool):
        self._model.update(operational=new_state)

    @property
    def location(self) -> Location:
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')

    @property
    def station_op_history(self) -> List[StationOpDescriptor]:
        self._model.reload('station_op_history')
        return [ObjectFactory.construct_station_op_from_model(op_model) for op_model in self._model.station_op_history]

    @property
    def requested_robot_op_history(self) -> List[Any]:
        self._model.reload('requested_robot_op_history')
        return [loads(encoded_robob_job) for encoded_robob_job in self._model.requested_robot_op_history]

    @property
    def process_sm_dict(self) -> dict:
        return self._model.process_state_machine

    @property
    def batch_capacity(self) -> int:
        return self._model.batch_capacity

    def load_sample(self):
        self._model.update(inc__loaded_samples=1)

    def unload_sample(self):
        self._model.update(dec__loaded_samples=1)


    @property
    def loaded_samples(self) -> int:
        self._model.reload('loaded_samples')
        return self._model.loaded_samples

    @property
    def assigned_batches(self) -> List[Batch]:
        self._model.reload('assigned_batches')
        assigned_batches_list = []
        if self._model.assigned_batches:
            assigned_batches_list = [Batch(batch_model) for batch_model in self._model.assigned_batches]
        return assigned_batches_list

    @property
    def processed_batches(self) -> List[Batch]:
        self._model.reload('processed_batches')
        processed_batches_list = []
        if self._model.processed_batches:
            processed_batches_list = [Batch(batch_model) for batch_model in self._model.processed_batches]
        return processed_batches_list 

    def has_free_batch_capacity(self) -> bool:
        return len(self.assigned_batches) < self.batch_capacity


    def add_batch(self, batch: Batch):
        if self.has_free_batch_capacity():
            self._model.update(push__assigned_batches=batch.model)
            self._log_station(f'{batch} is added for processing.')
            station_stamp = str(self)
            batch.add_station_stamp(station_stamp)
            if len(self.assigned_batches) == self.batch_capacity:
                self._update_state(StationState.PROCESSING)
        else:
            self._log_station(f'Cannot add {batch}, no batch capacity is available')

    def has_processed_batch(self) -> bool:
        self._model.reload('processed_batches')
        return True if self._model.processed_batches else False

    def process_assigned_batches(self):
        self._model.reload('assigned_batches')
        self._model.update(processed_batches=self._model.assigned_batches)
        self._model.update(unset__assigned_batches=True)
        self._log_station(f'all assigned batches processing complete.')
        if len(self.processed_batches) == self.batch_capacity:
            self._update_state(StationState.PROCESSING_COMPLETE)

    def get_processed_batch(self) -> Batch:
        processed_batches = self.processed_batches
        if processed_batches:
            batch = processed_batches.pop(0)
            self._model.update(pop__processed_batches=-1)
            self._log_station(f'Processed id:{batch} is unassigned.')
            if len(self.processed_batches) == 0:
                self._update_state(StationState.IDLE)
            return batch

    def has_robot_job(self) -> bool:
        self._model.reload('requested_robot_op')
        return self._model.requested_robot_op is not None
    
    def get_robot_job(self) -> StationRobotJob:
        if self.has_robot_job():
            robot_op = loads(self._model.requested_robot_op)
            self._log_station(f'Robot job request for ({robot_op}) is retrieved.')
            self._update_state(StationState.WAITING_ON_ROBOT)
            return robot_op

    def set_robot_job(self, robot_job, current_batch_id = -1):
        station_robot_job = StationRobotJob(robot_job, current_batch_id, self._model.id) 
        encoded_station_robot_job = dumps(station_robot_job)
        self._model.update(requested_robot_op=encoded_station_robot_job)
        self._log_station(f'Requesting robot job ({robot_job})')

    def finish_robot_job(self, complete_robot_op: RobotOpDescriptor):
        encodedRobotOp = dumps(complete_robot_op)
        self._model.update(unset__requested_robot_op=True)
        self._model.update(push__requested_robot_op_history=encodedRobotOp)
        self._log_station(f'Robot job request is fulfilled.')
        self._update_state(StationState.PROCESSING)

    def has_station_op(self):
        self._model.reload('current_station_op')
        return self._model.current_station_op is not None

    def get_station_op(self):
        if self.has_station_op():
            return ObjectFactory.construct_station_op_from_model(self._model.current_station_op)

    def set_station_op(self, station_op: StationOpDescriptor):
        self._model.update(current_station_op=station_op.model)
        self._update_state(StationState.WAITING_ON_OPERATION)
        self._log_station(f'Requesting station job ({station_op})')
        

    def finish_station_op(self, complete_station_op: StationOpDescriptor):
        self._model.update(unset__current_station_op=True)
        self._model.update(push__station_op_history=complete_station_op.model)
        self._log_station(f'Station op is complete.')
        self._update_state(StationState.PROCESSING)

    def create_location_from_frame(self, frame: str) -> Location:
        return Location(self.location.node_id, self.location.graph_id, frame)

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: StationState):
        self._model.update(state=new_state)
        self._log_station(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


