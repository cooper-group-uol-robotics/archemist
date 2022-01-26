from archemist.persistence.dbObjProxy import DbObjProxy
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from enum import Enum
from archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError
from datetime import datetime

class RobotState(Enum):
    EXECUTING_JOB = 0
    EXECUTION_COMPLETE = 1
    #WAITING_ON_STATION = 1
    IDLE = 2

class RobotOutputDescriptor:
    def __init__(self):
        self._success = False
        self._has_result = False
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

class RobotOpDescriptor:
    def __init__(self, output: RobotOutputDescriptor):
        self._output = output
        self._timestamp = None

    @property
    def output(self):
        return self._output

    @output.setter
    def output(self, value):
        if isinstance(value, RobotOutputDescriptor):
            self._output = value
        else:
            raise ValueError

    def add_timestamp(self):
        self._timestamp = datetime.now()

class MoveSampleOp(RobotOpDescriptor):
    def __init__(self, sample_index: int , start_location: Location, target_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._sample_index = sample_index
        self._start_location = start_location
        self._target_location = target_location

    @property
    def sample_index(self):
        return self._sample_index

    @property
    def start_location(self):
        return self._start_location

    @property
    def target_location(self):
        return self._target_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} {self._start_location.frame_name} -> {self._target_location.frame_name}, sample_index: {self._sample_index}'

class PickAndPlaceBatchOp(RobotOpDescriptor):
    def __init__(self, pick_location: Location, place_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._pick_location = pick_location
        self._place_location = place_location

    @property
    def pick_location(self):
        return self._pick_location

    @property
    def place_location(self):
        return self._place_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} {self._pick_location} -> {self._place_location}'

class PickBatchToDeckOp(RobotOpDescriptor):
    def __init__(self, pick_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._pick_location = pick_location

    @property
    def pick_location(self):
        return self._pick_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} {self._pick_location} -> robot_deck'

class PlaceBatchFromDeckOp(RobotOpDescriptor):
    def __init__(self, place_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._place_location = place_location

    @property
    def place_location(self):
        return self._place_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} robot_deck -> {self._place_location}'

class SpecialJobOpDescriptor(RobotOpDescriptor):
    def __init__(self, job_name: str, target_location: Location, output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._job_name = job_name
        self._job_location = target_location

    @property
    def job_name(self):
        return self._job_name

    @property
    def job_location(self):
        return self._job_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self._job_name} @{self._job_location}'

class Robot(DbObjProxy):
    def __init__(self, db_name: str, robot_document: dict):

        if len(robot_document) > 1:

            robot_document['operational'] = True
            robot_document['location'] = None
            
            robot_document['assigned_job'] = None
            robot_document['complete_job'] = None

            robot_document['state'] = RobotState.IDLE.value
            
            robot_document['robot_job_history'] = []
            super().__init__(db_name, 'robots', robot_document)
        else:
            super().__init__(db_name, 'robots', robot_document['object_id'])

    @property
    def id(self):
        return self.get_field('id')

    @property
    def saved_frames(self):
        return self.get_field('saved_frames')

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
        if loc_dict is not None:
            return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, value):
        if isinstance(value, Location):
            loc_dict = {'node_id':value.node_id, 'graph_id':value.graph_id, 'frame_name':value.frame_name}
            self.update_field('location', loc_dict)
        else:
            raise ValueError

    @property
    def state(self):
            return RobotState(self.get_field('state'))

    @property
    def assigned_job(self) -> StationRobotJob:
            encoded_job = self.get_field('assigned_job')
            if encoded_job is not None:
                return DbObjProxy.decode_object(encoded_job)

    @property
    def robot_job_history(self):
        en_op_history = self.get_field('robot_job_history')
        return [self.decode_object(op) for op in en_op_history]

    def assign_job(self, station_robot_job: StationRobotJob):
        if(self.assigned_job is None):
            encoded_job = DbObjProxy.encode_object(station_robot_job)
            self.update_field('assigned_job', encoded_job)
            self._log_robot(f'Job ({station_robot_job.robot_op}) is assigned.')
            self._update_state(RobotState.EXECUTING_JOB)

        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def complete_assigned_job(self, robot_op_output: RobotOutputDescriptor):
        station_robot_job = self.assigned_job
        station_robot_job.robot_op.output = robot_op_output
        encoded_station_robot_job = DbObjProxy.encode_object(station_robot_job)
        self.update_field('complete_job', encoded_station_robot_job)
        self.update_field('assigned_job', None)
        self._log_robot(f'Job ({station_robot_job.robot_op} is complete.')
        self.push_to_array_field('robot_job_history', encoded_station_robot_job)
        self._update_state(RobotState.EXECUTION_COMPLETE)

    def has_complete_job(self):
        return self.get_field('complete_job') is not None

    def get_complete_job(self):
        encoded_station_robot_job = self.get_field('complete_job')
        if encoded_station_robot_job is not None:
            station_robot_job = DbObjProxy.decode_object(encoded_station_robot_job)
            self.update_field('complete_job', None)
            self._log_robot(f'Job ({station_robot_job.robot_op}) is retrieved.')
            self._update_state(RobotState.IDLE)
            return station_robot_job

    def _log_robot(self, message: str):
        print(f'[{self}]: {message}')

    def _update_state(self, new_state: RobotState):
        self.update_field('state',new_state.value)
        self._log_robot(f'Current state changed to {new_state}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'

class mobileRobot(Robot):
    def __init__(self, db_name: str, robot_document: dict):
        if len(robot_document) > 1:
            robot_document['rack_holders'] = []
        
        super().__init__(db_name, robot_document)

    @property 
    def rack_holders(self):
        return self._rack_holders

class armRobot(Robot):
    def __init__(self, db_name: str, robot_document: dict):
        super().__init__(db_name, robot_document)