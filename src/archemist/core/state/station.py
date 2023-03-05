from typing import List, Any
from archemist.core.util.enums import StationState
from archemist.core.models.station_model import StationModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from archemist.core.util.location import Location
from archemist.core.state.robot import RobotOpDescriptor
from archemist.core.state.batch import Batch
from archemist.core.persistence.object_factory import StationFactory, RobotFactory

class Station:
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: dict, liquids: List[Liquid], solids: List[Solid]):
        pass

    @staticmethod
    def _set_model_common_fields(station_dict: dict, station_model: StationModel):
        station_model._type = station_dict['type']
        station_model.exp_id = station_dict['id']
        station_model.location = station_dict['location']
        station_model.batch_capacity = station_dict['batch_capacity']
        station_model.process_state_machine = station_dict['process_state_machine']
        station_model.selected_handler = station_dict['handler']

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
        return [StationFactory.create_op_from_model(op_model) for op_model in self._model.station_op_history]

    @property
    def requested_robot_op_history(self) -> List[Any]:
        self._model.reload('requested_robot_op_history')
        return [RobotFactory.create_op_from_model(robot_job_model) for robot_job_model in self._model.requested_robot_op_history]

    @property
    def process_sm_dict(self) -> dict:
        return self._model.process_state_machine

    @property
    def process_status(self) -> dict:
        self._model.reload('process_status')
        return self._model.process_status
    
    @process_status.setter
    def process_status(self, new_status: dict):
        self._model.update(process_status=new_status)


    @property
    def selected_handler_dict(self) -> dict:
        station_module = self._model._module.rsplit('.',1)[0]
        return {'type':self._model.selected_handler, 'module':station_module}

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
        # if len(self.processed_batches) == self.batch_capacity:
        #     self._update_state(StationState.PROCESSING_COMPLETE)

    def get_processed_batch(self) -> Batch:
        processed_batches = self.processed_batches
        if processed_batches:
            batch = processed_batches.pop(0)
            self._model.update(pop__processed_batches=-1)
            self._log_station(f'Processed id:{batch} is unassigned.')
            if len(self.processed_batches) == 0:
                self._update_state(StationState.IDLE)
            return batch

    def has_requested_robot_op(self) -> bool:
        self._model.reload('requested_robot_op')
        return self._model.requested_robot_op is not None
    
    def get_requested_robot_op(self) -> RobotOpDescriptor:
        if self.has_requested_robot_op():
            robot_op = RobotFactory.create_op_from_model(self._model.requested_robot_op)
            self._log_station(f'Robot job request for ({robot_op}) is retrieved.')
            self._update_state(StationState.WAITING_ON_ROBOT)
            return robot_op

    def request_robot_op(self, robot_job: RobotOpDescriptor, current_batch_id = -1):
        if current_batch_id != -1:
            robot_job.related_batch_id = current_batch_id
        robot_job.origin_station = self._model.id
        self._model.update(requested_robot_op=robot_job.model)
        self._log_station(f'Requesting robot job ({robot_job})')

    def complete_robot_op_request(self, complete_robot_op: RobotOpDescriptor):
        self._model.update(unset__requested_robot_op=True)
        self._model.update(push__requested_robot_op_history=complete_robot_op.model)
        self._log_station(f'Robot job request is fulfilled.')
        self._update_state(StationState.PROCESSING)

    def has_assigned_station_op(self):
        self._model.reload('assigned_station_op')
        return self._model.assigned_station_op is not None

    def get_assigned_station_op(self):
        if self.has_assigned_station_op():
            return StationFactory.create_op_from_model(self._model.assigned_station_op)

    def assign_station_op(self, station_op: StationOpDescriptor):
        self._model.update(assigned_station_op=station_op.model)
        self._update_state(StationState.OP_ASSIGNED)
        self._log_station(f'Requesting station job ({station_op})')
        

    def complete_assigned_station_op(self, success: bool, **kwargs):
        self._model.reload('assigned_station_op')
        station_op = StationFactory.create_op_from_model(self._model.assigned_station_op)
        station_op.complete_op(success, **kwargs)
        self._model.update(unset__assigned_station_op=True)
        self._model.update(push__station_op_history=station_op.model)
        self._log_station(f'Station op is complete.')

    def start_executing_op(self):
        op = self.get_assigned_station_op()
        op.add_start_timestamp()
        self._model.update(assigned_station_op=op.model)
        self._update_state(StationState.EXECUTING_OP)

    def set_to_processing(self):
        self._update_state(StationState.PROCESSING)

    def repeat_assigned_op(self):
        if self.has_assigned_station_op() and self.state == StationState.EXECUTING_OP:
            self._update_state(StationState.REPEAT_OP)
        else:
            self._log_station('Unable to repeat op since it is not under execution')

    def skip_assigned_op(self):
        if self.has_assigned_station_op() and self.state == StationState.EXECUTING_OP:
            self._update_state(StationState.SKIP_OP)
        else:
            self._log_station('Unable to skip op since it is not under execution')

    def create_location_from_frame(self, frame: str) -> Location:
        return Location(self.location.node_id, self.location.graph_id, frame)

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: StationState):
        self._model.update(state=new_state)
        self._log_station(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


