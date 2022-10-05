from bson.objectid import ObjectId
from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.persistence.dbObjProxy import DbObjProxy
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from archemist.state.robot import RobotOpDescriptor
from archemist.state.batch import Batch
#import archemist.processing.stationSMs

class StationState(Enum):
    IDLE = 0
    PROCESSING = 1
    WAITING_ON_ROBOT = 2
    WAITING_ON_OPERATION = 3
    OPERATION_COMPLETE = 4
    PROCESSING_COMPLETE = 5

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

class StationOpDescriptor:
    def __init__(self, stationName: str, output: StationOutputDescriptor):
        self._stationName = stationName
        self._output = output
        self._timestamp = None

    @property
    def stationName(self):
        return self._stationName

    @property
    def output(self):
        return self._output

    def add_timestamp(self):
        self._timestamp = datetime.now()


class Station(DbObjProxy):
    def __init__(self, db_name: str, station_document: dict):

        if len(station_document) > 1:
            
            station_document['operational'] = True

            station_document['state'] = StationState.IDLE.value

            station_document['assigned_batches'] = []
            station_document['processed_batches'] = []
            station_document['req_robot_job'] = None

            station_document['loaded_samples'] = 0        
            
            station_document['current_station_op'] = None
            station_document['station_op_history'] = []
            station_document['requested_robot_op_history'] = []
            
            super().__init__(db_name, 'stations', station_document)
        else:
            super().__init__(db_name, 'stations', station_document['object_id'])

        @classmethod
        def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
            pass

        @classmethod
        def from_object_id(cls, db_name: str, object_id: ObjectId):
            pass

    @property
    def state(self):
            return StationState(self.get_field('state'))

    @property
    def id(self):
        return self.get_field('id')

    @property
    def operational(self):
        return self.get_field('operational')

    @operational.setter
    def operational(self, value):
        if isinstance(value, bool):
            self.update_field('operational',value)
        else:
            raise ValueError

    @property
    def location(self):
        loc_dict = self.get_field('location')
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')

    @property
    def station_op_history(self):
        en_op_history = self.get_field('station_op_history')
        return [self.decode_object(op) for op in en_op_history]

    @property
    def requested_robot_op_history(self):
        en_op_history = self.get_field('requested_robot_op_history')
        return [self.decode_object(op) for op in en_op_history]

    @property
    def current_station_op(self):
        encoded_op = self.get_field('current_station_op')
        if encoded_op is not None:
            return self.decode_object(encoded_op)

    @property
    def process_sm_dict(self):
        return self.get_field('process_state_machine')

    @property
    def batch_capacity(self):
        return self.get_field('batch_capacity')

    def load_sample(self):
        self.increment_field('loaded_samples')

    def unload_sample(self):
        self.decrement_field('loaded_samples')

    @property
    def loaded_samples(self):
        return self.get_field('loaded_samples')

    @property
    def assigned_batches(self):
        batch_obj_id_list = self.get_field('assigned_batches')
        if len(batch_obj_id_list) > 0:
            return [Batch.from_object_id(self.db_name, batch_obj_id) for batch_obj_id in batch_obj_id_list]
        else:
            return []

    def has_free_batch_capacity(self):
        return len(self.get_field('assigned_batches')) < self.batch_capacity


    def add_batch(self, batch):
        if len(self.get_field('assigned_batches')) < self.batch_capacity:
            self.push_to_array_field('assigned_batches', batch.object_id)
            self._log_station(f'{batch} is added for processing.')
            station_stamp = str(self)
            batch.add_station_stamp(station_stamp)
            if len(self.get_field('assigned_batches')) == self.batch_capacity:
                self.set_to_processing()
        else:
            raise exception.StationAssignedRackError(self.__class__.__name__)

    def has_processed_batch(self):
        return len(self.get_field('processed_batches')) > 0

    def process_assigned_batches(self):
        batch_obj_id_list = self.get_field('assigned_batches')
        for batch_obj_id in batch_obj_id_list:
            self.push_to_array_field('processed_batches',batch_obj_id)
            self.delete_element_from_array_field('assigned_batches',batch_obj_id)
            self._log_station(f'Processing batch is complete.')
        if len(self.get_field('processed_batches')) == self.batch_capacity:
            self._update_state(StationState.PROCESSING_COMPLETE)

    def get_processed_batch(self):
        batch_obj_id = self.get_doc_from_array_field('processed_batches', 0)
        if batch_obj_id is not None:
            batch = Batch.from_object_id(self.db_name, batch_obj_id)
            self.delete_element_from_array_field('processed_batches', batch_obj_id)
            self._log_station(f'Processed id:{batch} is unassigned.')
            if len(self.get_field('processed_batches')) == 0:
                self._update_state(StationState.IDLE)
            return batch

    def has_robot_job(self):
        return self.get_field('req_robot_job') is not None
    
    def get_robot_job(self):
        encoded_robot_job = self.get_field('req_robot_job')
        if encoded_robot_job is not None:
            robot_job = DbObjProxy.decode_object(encoded_robot_job)
            self._log_station(f'Robot job request for ({robot_job}) is retrieved.')
            self._update_state(StationState.WAITING_ON_ROBOT)
            return robot_job

    def set_robot_job(self, robot_job, current_batch_id = -1):
        station_robot_job = StationRobotJob(robot_job, current_batch_id, self.object_id) 
        encoded_station_robot_job = DbObjProxy.encode_object(station_robot_job)
        self.update_field('req_robot_job', encoded_station_robot_job)
        self._log_station(f'Requesting robot job ({robot_job})')

    def finish_robot_job(self, complete_robot_op: RobotOpDescriptor):
        encodedRobotOp = DbObjProxy.encode_object(complete_robot_op)
        self.update_field('req_robot_job', None)
        self._log_station(f'Robot job request is fulfilled.')
        self.push_to_array_field('requested_robot_op_history',encodedRobotOp)
        self.set_to_processing()

    def has_station_op(self):
        return self.get_field('current_station_op') is not None

    def get_station_op(self):
        encoded_op = self.get_field('current_station_op')
        if encoded_op is not None:
            station_op = DbObjProxy.decode_object(encoded_op)
            return station_op

    def set_station_op(self, stationOp: StationOpDescriptor):
        encodedStationOp = DbObjProxy.encode_object(stationOp)
        self.update_field('current_station_op', encodedStationOp)
        self.request_station_operation()
        self._log_station(f'Requesting station job ({stationOp})')

    def finish_station_op(self, complete_station_op: StationOpDescriptor):
        encodedStationOp = DbObjProxy.encode_object(complete_station_op)
        self.update_field('current_station_op', None)
        self._log_station(f'Station op is complete.')
        self.push_to_array_field('station_op_history',encodedStationOp)
        self.set_to_processing()

    def set_to_processing(self):
        self._update_state(StationState.PROCESSING)

    def request_station_operation(self):
        self._update_state(StationState.WAITING_ON_OPERATION)

    def finish_station_operation(self):
        self._update_state(StationState.OPERATION_COMPLETE)

    def create_location_from_frame(self, frame: str) -> Location:
        return Location(self.location.node_id, self.location.graph_id, frame)

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: StationState):
        self.update_field('state',new_state.value)
        self._log_station(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


