from bson.objectid import ObjectId
from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.persistence.dbObjProxy import DbObjProxy
from archemist.util.location import Location
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
            
            station_document['object'] = self.__class__.__name__
            station_document['operational'] = True

            station_document['state'] = StationState.IDLE.value

            station_document['assigned_batch'] = None
            station_document['processed_batch'] = None
            station_document['req_robot_job'] = None

            station_document['loaded_samples'] = 0        
            
            station_document['current_station_op'] = None
            station_document['station_op_history'] = []
            
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

    def set_station_op(self, stationOp: StationOpDescriptor):
        encodedStationOp = self.encode_object(stationOp)
        self.update_field('current_station_op', encodedStationOp)
        self.push_to_array_field('station_op_history',encodedStationOp)

    @property
    def station_op_history(self):
        en_op_history = self.get_field('station_op_history')
        return [self.decode_object(op) for op in en_op_history]

    @property
    def current_station_op(self):
        encoded_op = self.get_field('current_station_op')
        if encoded_op is not None:
            return self.decode_object(encoded_op)

    @property
    def process_sm_dict(self):
        return self.get_field('process_state_machine')

    def load_sample(self):
        self.increment_field('loaded_samples')

    def unload_sample(self):
        self.decrement_field('loaded_samples')

    @property
    def loaded_samples(self):
        return self.get_field('loaded_samples')

    @property
    def assigned_batch(self):
        batch_obj_id = self.get_field('assigned_batch')
        if batch_obj_id is not None:
            return Batch.from_objectId(self.db_name, batch_obj_id)


    def add_batch(self, batch):
        if(self.assigned_batch is None):
            self.update_field('assigned_batch', batch.object_id)
            self._log_station(f'{batch} is assigned for processing.')
            station_stamp = str(self)
            batch.add_station_stamp(station_stamp)
            self._update_state(StationState.PROCESSING)
        else:
            raise exception.StationAssignedRackError(self.__class__.__name__)

    def has_processed_batch(self):
        return self.get_field('processed_batch') is not None

    def process_assigned_batch(self):
        self.update_field('processed_batch', self.get_field('assigned_batch'))
        self.update_field('assigned_batch',None)
        self._log_station(f'Processing batch is complete.')
        self._update_state(StationState.PROCESSING_COMPLETE)

    def get_processed_batch(self):
        batch_obj_id = self.get_field('processed_batch')
        if batch_obj_id is not None:
            batch = Batch.from_objectId(self.db_name, batch_obj_id)
            self.update_field('processed_batch', None)
            self._log_station(f'Processed id:{batch} is unassigned.')
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

    def set_robot_job(self, robot_job):
        encoded_robot_job = DbObjProxy.encode_object(robot_job)
        self.update_field('req_robot_job', encoded_robot_job)
        self._log_station(f'Requesting robot job ({robot_job})')

    def finish_robot_job(self):
        self.update_field('req_robot_job', None)
        self._log_station(f'Robot job request is fulfilled.')
        self._update_state(StationState.PROCESSING)

    def finish_station_operation(self):
        self._update_state(StationState.OPERATION_COMPLETE)

    def create_location_from_frame(self, frame: str) -> Location:
        return Location(self._location.node_id, self._location.graph_id, frame)

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: StationState):
        self.update_field('state',new_state.value)
        self._log_station(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'


