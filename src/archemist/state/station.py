from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.util.location import Location
#from archemist.state.batch import Batch, Sample
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


class Station:
    def __init__(self, id: int, location:Location):
        self._id = id

        self._operational = False
        self._state = StationState.IDLE
        self._location = location

        self._assigned_batch = None
        self._processed_batch = None
        self._req_robot_job = None

        self._loaded_samples = 0        
        
        self._current_station_op = None
        self._station_op_history = []

    def set_station_op(self, stationOp: StationOpDescriptor):
        self._current_station_op = stationOp
        self._station_op_history = self._station_op_history.append(stationOp)

    @property
    def state(self):
            return self._state

    @state.setter
    def state(self, new_state):
        if isinstance(new_state, StationState):
            self._state = new_state
            self._log_station(f'Current state changed to {self._state}')
        else:
            raise ValueError

    @property
    def id(self):
        return self._id

    @property
    def operational(self):
        return self._operational

    @property
    def location(self):
        return self._location

    @operational.setter
    def operational(self, value):
        if isinstance(value, bool):
            self._operational = value
        else:
            raise ValueError

    @property
    def station_op_history(self):
        return self._station_op_history

    def load_sample(self):
        self._loaded_samples += 1

    def unload_sample(self):
        self._loaded_samples -= 1

    @property
    def loaded_samples(self):
        return self._loaded_samples

    @property
    def assigned_batch(self):
        return self._assigned_batch


    def add_batch(self, batch):
        if(self._assigned_batch is None):
            self._assigned_batch = batch
            self._log_station(f'{batch} is assigned for processing.')
            station_stamp = str(self)
            batch.add_station_stamp(station_stamp)
            self._state = StationState.PROCESSING
        else:
            raise exception.StationAssignedRackError(self.__class__.__name__)

    def has_processed_batch(self):
        return self._processed_batch is not None

    def process_assigned_batch(self):
        self._processed_batch = self._assigned_batch
        self._assigned_batch = None
        self._log_station(f'Processing {self._processed_batch} is complete.')
        self.state = StationState.PROCESSING_COMPLETE

    def get_processed_batch(self):
        batch = self._processed_batch
        if self._processed_batch is not None: 
            self._processed_batch = None
            self._log_station(f'Processed id:{batch} is unassigned.')
            self._state = StationState.IDLE
        return batch

    def has_robot_job(self):
        return self._req_robot_job is not None
    
    def get_robot_job(self):
        self._log_station(f'Robot job request for ({self._req_robot_job}) is retrieved.')
        self._state = StationState.WAITING_ON_ROBOT
        return self._req_robot_job

    def set_robot_job(self, robot_job):
        self._req_robot_job = robot_job
        self._log_station(f'Requesting robot job ({self._req_robot_job})')

    def finish_robot_job(self):
        self._req_robot_job = None
        self._log_station(f'Robot job request is fulfilled.')
        self._state = StationState.PROCESSING

    def create_location_from_frame(self, frame: str) -> Location:
        return Location(self._location.node_id, self._location.graph_id, frame)

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self._id}'


