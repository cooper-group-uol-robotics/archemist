from typing import List, Any, Dict
from archemist.core.util.enums import StationState, OpState
from archemist.core.models.station_model import StationModel, StationProcessDataModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from archemist.core.util.location import Location
from archemist.core.state.robot import RobotOpDescriptor
from archemist.core.state.batch import Batch
from archemist.core.persistence.object_factory import StationFactory, RobotFactory
from archemist.core.util.list_field_adapter import EmbedOpListAdapter, EmbedListFieldAdapter
from bson.objectid import ObjectId
import uuid

class StationProcessData:
    def __init__(self, process_data_model: StationProcessDataModel) -> None:
        self._model = process_data_model

    @classmethod
    def from_args(cls, batches: List[Batch], processing_slot: int=0):
        model = StationProcessDataModel()
        model.uuid = uuid.uuid4()
        model.batches = [batch.model for batch in batches]
        model.status['state'] = 'init_state'
        model.processing_slot = processing_slot
        return cls(model)

    @property
    def model(self) -> StationProcessDataModel:
        return self._model

    @property
    def uuid(self) -> uuid.UUID:
        return self._model.uuid
    
    @property
    def processing_slot(self) -> int:
        return self._model.processing_slot

    @property
    def batches(self) -> List[Batch]:
        return [Batch(batch_model) for batch_model in self._model.batches]
    
    @property
    def status(self) -> dict:
        return self._model.status
    
    @property
    def req_robot_ops(self) -> List[Any]:
        return EmbedOpListAdapter(self._model, 'req_robot_ops', RobotFactory)
    
    @property
    def req_station_ops(self) -> List[Any]:
        return EmbedOpListAdapter(self._model, 'req_station_ops', StationFactory)
    
    @property
    def robot_ops_history(self) -> List[str]:
        return EmbedListFieldAdapter(self._model, 'robot_ops_history')
    
    @property
    def station_ops_history(self) -> List[str]:
        return EmbedListFieldAdapter(self._model, 'station_ops_history')


class Station:
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @staticmethod
    def _set_model_common_fields(station_dict: dict, station_model: StationModel):
        station_model._type = station_dict['type']
        station_model.exp_id = station_dict['id']
        station_model.location = station_dict['location']
        station_model.batch_capacity = station_dict['batch_capacity']
        station_model.process_batch_capacity = station_dict['process_batch_capacity']
        station_model.process_state_machine = station_dict['process_state_machine']
        station_model.selected_handler = station_dict['handler']

    ''' General properties and methods'''

    @property
    def model(self) -> StationModel:
        self._model.reload()
        return self._model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model.id


    @property
    def state(self) -> StationState:
        self._model.reload('state')
        return self._model.state

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def location(self) -> Location:
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    @property
    def selected_handler_dict(self) -> dict:
        station_module = self._model._module.rsplit('.',1)[0]
        return {'type':self._model.selected_handler, 'module':station_module}

    @property
    def batch_capacity(self) -> int:
        return self._model.batch_capacity
    
    ''' Process properties and methods '''

    @property
    def process_sm_dict(self) -> dict:
        return self._model.process_state_machine
    
    @property
    def process_batch_capacity(self) -> int:
        return self._model.process_batch_capacity

    
    def get_all_processes_data(self) -> List[StationProcessData]:
        self._model.reload('process_data_map')
        return [StationProcessData(process_data) for process_data in self._model.process_data_map.values()]
    
    def get_process_data(self, process_uuid: uuid.UUID) -> StationProcessData:
        self._model.reload('process_data_map')
        process_data_model = self._model.process_data_map.get(str(process_uuid))
        if process_data_model is not None:
            return StationProcessData(process_data_model)
    
    def set_process_data(self, process_data: StationProcessData):
        return self._model.update(**{f"set__process_data_map__{process_data.uuid}":process_data.model})
    
    def delete_process_data(self, process_uuid: uuid.UUID):
        self._model.update(**{f"unset__process_data_map__{process_uuid}":True})

    def get_batch_process_uuid(self, batch: Batch) -> uuid.UUID:
        for process in self.get_all_processes_data():
            if batch in process.batches:
                return process.uuid

    ''' Batches properties and methods '''

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
        else:
            self._log_station(f'Cannot add {batch}, no batch capacity is available')

    def has_processed_batch(self) -> bool:
        self._model.reload('processed_batches')
        return True if self._model.processed_batches else False
    
    def process_assinged_batch(self, batch: Batch):
        self._model.update(push__processed_batches=batch.model)
        self._model.reload('assigned_batches')
        self._model.update(pull__assigned_batches=batch.model)
        self._log_station(f'batch {batch.id} processed.')

    def process_assigned_batches(self):
        self._model.reload('assigned_batches')
        self._model.update(processed_batches=self._model.assigned_batches)
        self._model.update(unset__assigned_batches=True)
        self._log_station(f'all assigned batches processing complete.')

    def get_processed_batch(self) -> Batch:
        processed_batches = self.processed_batches
        if processed_batches:
            batch = processed_batches.pop(0)
            self._model.update(pop__processed_batches=-1)
            self._log_station(f'Processed id:{batch} is unassigned.')
            return batch
        
    ''' Robot ops properties and methods '''

    @property
    def completed_robot_ops(self) -> Dict[Any,Any]:
        self._model.reload('completed_robot_ops')
        return {op_uuid:  RobotFactory.create_op_from_model(op_model) for op_uuid, op_model in self._model.completed_robot_ops.items()}

    def has_requested_robot_ops(self) -> bool:
        self._model.reload('requested_robot_op')
        return bool(self._model.requested_robot_op)
    
    def get_requested_robot_ops(self) -> List[RobotOpDescriptor]:
        robot_ops = []
        if self.has_requested_robot_ops():
            robot_ops = [RobotFactory.create_from_model(op_model) for op_model in self._model.requested_robot_op]
            self._model.update(set__requested_robot_op=[])
            self._log_station(f'Robot job request for ({robot_ops}) is retrieved.')
        return robot_ops

    def request_robot_op(self, robot_job: RobotOpDescriptor, current_batch_id = -1):
        if current_batch_id != -1:
            robot_job.related_batch_id = current_batch_id
        robot_job.origin_station = self._model.id
        self._model.update(push__requested_robot_op= robot_job.model)
        self._log_station(f'Requesting robot job ({robot_job})')

    def complete_robot_op_request(self, complete_robot_op: RobotOpDescriptor):
        self._model.update(**{f"set__completed_robot_ops__{complete_robot_op.uuid}":complete_robot_op.model})
        self._log_station(f'Robot job request is fulfilled.')

    ''' Station ops properties and methods '''

    @property
    def completed_station_ops(self) -> Dict[Any,Any]:
        self._model.reload('completed_station_ops')
        return {op_uuid:  StationFactory.create_op_from_model(op_model) for op_uuid, op_model 
                in self._model.completed_station_ops.items()}
    
    @property
    def assigned_op_state(self) -> OpState:
        self._model.reload("assigned_op_state")
        return self._model.assigned_op_state
    
    @assigned_op_state.setter
    def assigned_op_state(self, new_state: OpState):
        self._model.update(assigned_op_state=new_state)
    
    def has_queued_ops(self):
        self._model.reload('queued_ops')
        return bool(self._model.queued_ops)
    
    def has_assigned_station_op(self):
        self._model.reload('assigned_op')
        return self._model.assigned_op is not None
    
    def update_assigned_op(self):
        if self.has_queued_ops() and not self.has_assigned_station_op():
            op_model = self._model.queued_ops[0]
            self._model.update(set__assigned_op=op_model)
            self.assigned_op_state = OpState.ASSIGNED
            self._model.update(pop__queued_ops=-1)

    def get_assigned_station_op(self):
        if self.has_assigned_station_op():
            return StationFactory.create_op_from_model(self._model.assigned_op)

    def assign_station_op(self, station_op: StationOpDescriptor):
        if station_op.requested_by is None:
            station_op.add_request_info(self.object_id)
        self._model.update(push__queued_ops=station_op.model)
        self._log_station(f'Requesting station op ({station_op})')  

    def complete_assigned_station_op(self, success: bool, **kwargs):
        self._model.reload('assigned_op')
        station_op = StationFactory.create_op_from_model(self._model.assigned_op)
        station_op.complete_op(success, **kwargs)
        self._model.update(unset__assigned_op=True)
        self.assigned_op_state = OpState.INVALID
        self._model.update(**{f"set__completed_station_ops__{station_op.uuid}":station_op.model})
        self._log_station(f'Station op is complete.')

    def add_timestamp_to_assigned_op(self):
        op = self.get_assigned_station_op()
        op.add_start_timestamp()
        self._model.update(assigned_op=op.model)

    def repeat_assigned_op(self):
        if self.has_assigned_station_op():
            self.assigned_op_state = OpState.TO_BE_REPEATED
        else:
            self._log_station('Unable to repeat. No op assigned')

    def skip_assigned_op(self):
        if self.has_assigned_station_op():
            self.assigned_op_state = OpState.TO_BE_SKIPPED
        else:
            self._log_station('Unable to skip. No op assigned')

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


