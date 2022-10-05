from archemist.persistence.dbObjProxy import DbObjProxy
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from enum import Enum
from archemist.exceptions.exception import RobotAssignedRackError, RobotUnAssignedRackError
from datetime import datetime

class RobotState(Enum):
    JOB_ASSIGNED = 0
    EXECUTING_JOB = 1
    EXECUTION_COMPLETE = 2
    #WAITING_ON_STATION = 1
    IDLE = 3

class RobotTaskType(Enum):
    LOAD_TO_ROBOT = 0
    UNLOAD_FROM_ROBOT = 1
    MANIPULATION = 2
    OTHER = 3

class RobotOutputDescriptor:
    def __init__(self):
        self._success = False
        self._has_result = False
        self._timestamp = None
        self._executing_robot = None

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

    @property
    def executing_robot(self):
        return self._executing_robot

    @executing_robot.setter
    def executing_robot(self, robot_stamp):
        self._executing_robot = robot_stamp

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
    def __init__(self, task_name: str ,sample_index: int , output: RobotOutputDescriptor):
        super().__init__(output=output)
        self._task_name = task_name
        self._sample_index = sample_index

    @property
    def task_name(self):
        return self._task_name

    @property
    def sample_index(self):
        return self._sample_index

    def __str__(self) -> str:
        return f'{self.__class__.__name__} task_name: {self._task_name}, sample_index: {self._sample_index}'
    @property
    def pick_location(self):
        return self._pick_location

    @property
    def place_location(self):
        return self._place_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} {self._pick_location} -> {self._place_location}'

class RobotTaskOpDescriptor(RobotOpDescriptor):
    def __init__(self, job_name: str, job_params: list, job_type: RobotTaskType, job_location: Location, output: RobotOutputDescriptor = RobotOutputDescriptor()):
        super().__init__(output=output)
        self._job_name = job_name
        self._job_type = job_type
        self._job_params = job_params
        self._job_location = job_location

    @property
    def job_name(self):
        return self._job_name

    @property
    def job_type(self):
        return self._job_type

    @property
    def job_params(self):
        return self._job_params

    @property
    def job_location(self):
        return self._job_location

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self._job_name} @{self._job_location}'

class Robot(DbObjProxy):
    def __init__(self, db_name: str, robot_document: dict):

        if len(robot_document) > 1:

            robot_document['operational'] = True
            if 'location' not in robot_document:
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
            self._update_state(RobotState.JOB_ASSIGNED)

        else:
            raise RobotAssignedRackError(self.__class__.__name__)

    def start_job_execution(self):
        self._update_state(RobotState.EXECUTING_JOB)

    def complete_assigned_job(self, station_robot_job: StationRobotJob):
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
        return f'{self.__class__.__name__}_{self.id}'

class MobileRobot(Robot):
    def __init__(self, db_name: str, robot_document: dict):
        if len(robot_document) > 1:
            robot_document['onboard_batches'] = []
        
        super().__init__(db_name, robot_document)

    @property
    def batch_capacity(self):
        return self.get_field('batch_capacity')

    @property 
    def onboard_batches(self):
        return self.get_field('onboard_batches')

    def add_to_onboard_batches(self, batch_id: int):
        if len(self.onboard_batches) < self.batch_capacity:
            self.push_to_array_field('onboard_batches',batch_id)
        else:
            self._log_robot(f'Cannot add batch {batch_id} to deck. Batch capacity exceeded')

    def remove_from_onboard_batches(self, batch_id:int):
        self.delete_element_from_array_field('onboard_batches',batch_id)

    def is_onboard_capacity_full(self):
        return len(self.onboard_batches) == self.batch_capacity

    def is_batch_onboard(self, batch_id: int):
        return batch_id in self.onboard_batches

    def get_complete_job(self):
        encoded_station_robot_job = self.get_field('complete_job')
        if encoded_station_robot_job is not None:
            station_robot_job = DbObjProxy.decode_object(encoded_station_robot_job)
            robot_op = station_robot_job.robot_op
            batch_id = station_robot_job.batch_id
            if isinstance(robot_op, RobotTaskOpDescriptor):
                if robot_op.job_type == RobotTaskType.LOAD_TO_ROBOT:
                    self.add_to_onboard_batches(batch_id)
                elif robot_op.job_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                    self.remove_from_onboard_batches(batch_id)
            self.update_field('complete_job', None)
            self._log_robot(f'Job ({station_robot_job.robot_op}) is retrieved.')
            self._update_state(RobotState.IDLE)
            return station_robot_job

class armRobot(Robot):
    def __init__(self, db_name: str, robot_document: dict):
        super().__init__(db_name, robot_document)