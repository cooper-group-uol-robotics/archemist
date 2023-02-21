#test comment

from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import WaitingOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location

class WaitingStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        self.operation_complete = False
        self._current_batch_index = 0
        self._current_batches_count = 0
        # all the variables below to be updated as global variables / added to models
        self._current_batch_capacity = 3 #Number of batches in the current set ot the number of batches the robot is going to load 
        self._station_total_capacity = 9 #Total batches the waiting station can hold
        self._station_batches_occupied = 0 #Occupied batches in the waiting station
        self.spots_available = self._station_total_capacity - self._station_batches_occupied

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='navigate_to_WaitingStation', on_enter=['request_navigate_to_WaitingStation', '_print_state']),
            State(name='retreat_from_WaitingStation', on_enter=['request_navigate_to_WaitingStation', '_print_state']),
            State(name='Waiting_process', on_enter=['request_process_operation', '_print_state']),
            State(name='load_batch', on_enter=['request_load_batch', '_print_state']),
            State(name='added_batch_update', on_enter=['update_loaded_batch_waiting', '_print_state']),
            State(name='unload_batch', on_enter=['request_unload_batch', '_print_state']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch_waiting', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'navigate_to_WaitingStation', 'conditions':['all_current_batches_assigned', 'is_station_job_ready']},
            {'trigger':self._trigger_function,'source':'navigate_to_WaitingStation','dest':'load_batch', 'unless':'is_station_operation_complete','conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'load_batch','dest':'added_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'load_batch', 'unless':'are_all_current_batches_loaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'retreat_from_WaitingStation', 'unless':'is_station_operation_complete','conditions':['is_station_job_ready','are_all_current_batches_loaded']},
            {'trigger':self._trigger_function, 'source':'retreat_from_WaitingStation','dest':'Waiting_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'Waiting_process','dest':'unload_batch', 'conditions':['is_station_operation_complete','is_station_job_ready'], 'before':'process_batches'},
            #is 'before' attribute required in previous transition
            {'trigger':self._trigger_function, 'source':'unload_batch','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'unload_batch', 'unless':'are_all_batches_unloaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'final_state', 'conditions':['is_station_job_ready','are_all_batches_unloaded','is_station_operation_complete']},

        ]

        if not self.spots_available < self.current_batch_capacity:
            self.init_state_machine(states=states, transitions=transitions)
            self._station_batches_occupied +=3


    
    def update_loaded_batch_waiting(self):
        self.update_batch_loc_to_station()
        self._current_batches_count += 1
        if self._current_batches_count == self._station.batch_capacity:
            self._current_batch_index = 0
        else:
            self._current_batch_index += 1

    def update_unloaded_batch_waiting(self):
        self.update_batch_loc_to_robot()
        self._current_batches_count -= 1
        if self._current_batches_count == self._station.batch_capacity:
            self._current_batch_index = 0
        else:
            self._current_batch_index += 1

    def all_current_batches_assigned(self) -> bool:
        if len(self._station.assigned_batches) < self._current_batch_capacity:
            return False
    
    def are_all_current_batches_loaded(self):
        return self._current_batches_count == self._station.batch_capacity

    def are_all_batches_unloaded(self):
        return self._current_batches_count == 0

    def is_station_operation_complete(self):
        return self.operation_complete

    def request_load_batch(self):
        robot_job = KukaLBRTask.from_args(name='LoadWaitingStation',params=[True,self._current_batch_index+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_batch(self):
        robot_job = KukaLBRTask.from_args(name='UnloadWaitingStation',params=[False,self._current_batch_index+1],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)
        

    def request_navigate_to_WaitingStation(self):
        self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config && replace the location information with waiting station 


    def request_process_operation(self):
        self._station.assign_station_op(WaitingOpDescriptor.from_args())
        self._Batch_waiting_start_timeStamp = datetime.now()
        self._preset_waiting = timedelta(hours=12)

        # current_op = self._station.assigned_batches[-1].recipe.get_current_task_op()
        # if isinstance (current_op,CSCSVJobOpDescriptor):
        #     contacnated_csv = ''
        #     for batch in self._station.assigned_batches:
        #         current_op = batch.recipe.get_current_task_op()
        #         contacnated_csv += current_op.csv_string
        #     current_op.csv_string = contacnated_csv
        #     self._station.assign_station_op(current_op)
        # elif isinstance(current_op, CSProcessingOpDescriptor):
        #     self._station.assign_station_op(current_op)


    def process_batches(self):
        self._current_time = datetime.now()
        self._waiting_duration = self._current_time - self._Batch_waiting_start_timeStamp

        if self._waiting_duration >= self._preset_waiting:
            return self.operation_complete = True
        ################## to check if the waiting          
        # last_operation_op = self._station.station_op_history[-1]
        # for batch in self._station.assigned_batches:
        #     for _ in range(0, batch.num_samples):
        #             batch.add_station_op_to_current_sample(last_operation_op)
        #             batch.process_current_sample()


    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



