from archemist.exceptions import exception
from enum import Enum
from datetime import datetime
from archemist.util.location import Location
from archemist.state.batch import Batch, Sample
from transitions import Machine

class StationState(Enum):
    WAITING_ON_ROBOT = 0
    PROCESSING = 1
    PROCESSING_COMPLETE = 2
    IDLE = 3

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
    def __init__(self, id: int, location:Location, process_sm: Machine):
        self._id = id

        self._operational = False
        self._state = StationState.IDLE
        self._process_sm = process_sm
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

    @property
    def id(self):
        return self._id

    @property
    def process_sm(self):
        return self._process_sm

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


    def add_batch(self, batch: Batch):
        if(self._assigned_batch is None):
            self._assigned_batch = batch
            print('Batch {id} assigned to station {name}'.format(id=batch.id,
                  name=self.__class__.__name__))
            self._state = StationState.PROCESSING
        else:
            raise exception.StationAssignedRackError(self.__class__.__name__)

    def has_processed_batch(self):
        return self._processed_batch is not None

    def batch_completed(self):
        self._processed_batch = self._assigned_batch
        self._assigned_batch = None

    def get_processed_batch(self):
        sample = self._processed_batch
        if self._processed_batch is not None: 
            self._processed_batch = None
            self._state = StationState.IDLE
        return sample

    def has_robot_job(self):
        return self._req_robot_job is not None
    
    def get_robot_job(self):
        return self._req_robot_job

    def set_robot_job(self, robot_job):
        self._req_robot_job = robot_job
        self._state = StationState.WAITING_ON_ROBOT

    def robot_job_done(self):
        self._req_robot_job = None
        self._state = StationState.IDLE


