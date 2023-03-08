
from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import WaitingStation, WaitingOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location

class WaitingStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False
        if '_current_batch_index' not in self._status.keys():
            self._status['_current_batch_index'] = 0
        if '_current_batches_count' not in self._status.keys():
            self._status['_current_batches_count'] = 0
        if '_preset_waiting' not in self._status.keys():
            self._status['_preset_waiting'] = timedelta(minutes=2)
        if '_current_batch_capacity' not in self._status.keys():
            self._status['_current_batch_capacity'] = 3
        if '_station_total_capacity' not in self._status.keys():
            self._status['_station_total_capacity'] = 9
        if '_station_batches_occupied' not in self._status.keys():
            self._status['_station_batches_occupied'] = 0


        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions', '_print_state']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions', '_print_state']),
            State(name='Waiting_process', on_enter=['request_process_operation', '_print_state']),
            State(name='load_batch', on_enter=['request_load_batch', '_print_state']),
            State(name='added_batch_update', on_enter=['update_loaded_batch_waiting', '_print_state']),
            State(name='unload_batch', on_enter=['request_unload_batch', '_print_state']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch_waiting', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'disable_auto_functions'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'load_batch', 'unless':'is_station_operation_complete','conditions':['is_station_job_ready', 'all_current_batches_assigned']},
            {'trigger':self._trigger_function, 'source':'load_batch','dest':'added_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'load_batch', 'unless':'are_all_current_batches_loaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'Waiting_process', 'unless':'is_station_operation_complete','conditions':['is_station_job_ready','are_all_current_batches_loaded']},
            {'trigger':self._trigger_function, 'source':'Waiting_process','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'disable_auto_functions','conditions':['is_station_operation_complete','is_station_job_ready']},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions','dest':'unload_batch','conditions':['is_station_operation_complete','is_station_job_ready']},
            {'trigger':self._trigger_function, 'source':'unload_batch','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'unload_batch', 'unless':'are_all_batches_unloaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'unload_batch','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','are_all_batches_unloaded','is_station_operation_complete']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready','are_all_batches_unloaded','is_station_operation_complete']}
        ]   


        self.init_state_machine(states=states, transitions=transitions)

    def update_loaded_batch_waiting(self):
        self.update_batch_loc_to_station()
        self._status['_current_batches_count'] += 1
        if self._status['_current_batches_count'] == self._station.batch_capacity:
            WaitingStation.update_batch_count(self._status['_current_batches_count'])
            self._status['_current_batch_index'] = 0
        else:
            self._status['_current_batch_index'] += 1

    def update_unloaded_batch_waiting(self):
        self.update_batch_loc_to_robot()
        self._status['_current_batches_count'] -= 1
        if self._status['_current_batches_count'] == 0:
            WaitingStation.update_batch_count(self._status['_current_batches_count'])
        else:
            pass

    def all_current_batches_assigned(self) -> bool:
        if len(self._station.assigned_batches) < self._status['_current_batch_capacity']:
            return False
    
    def are_all_current_batches_loaded(self):
        return self._status['_current_batches_count'] == self._status['_current_batch_capacity']

    def are_all_batches_unloaded(self):
        return self._status['_current_batches_count'] == 0

    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_load_batch(self):
        robot_job = KukaLBRTask.from_args(name='LoadWaitingStation',params=[True,self._status['_current_batch_index']+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['_current_batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)


    def request_unload_batch(self):
        robot_job = KukaLBRTask.from_args(name='UnloadWaitingStation',params=[False,self._status['_current_batch_index']+1],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['_current_batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)
        
    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def request_process_operation(self):
        self._station.assign_station_op(WaitingOpDescriptor.from_args())

        
    # def process_batches(self):
    #     self._current_time = datetime.now()
    #     self._waiting_duration = self._current_time - self._Batch_waiting_start_timeStamp
    #     if self._waiting_duration >= self._preset_waiting:
    #         self.operation_complete = True
    #     ################## to check if the waiting          
    #     # last_operation_op = self._station.station_op_history[-1]
    #     # for batch in self._station.assigned_batches:
    #     #     for _ in range(0, batch.num_samples):
    #     #             batch.add_station_op_to_current_sample(last_operation_op)
    #     #             batch.process_current_sample()


    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['operation_complete'] = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



