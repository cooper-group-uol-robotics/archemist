from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from .state import OptimaxStation, OptimaxTempStirringOpDescriptor, OptimaxTempOpDescriptor, OptimaxStirringOpDescriptor
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaNAVTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.util import Location

class WeighingSM(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'loaded_samples' not in self._status.keys():
            self._status['loaded_samples'] = 0

        ''' States '''
        states = [State(name='init_state'), 
            State(name='station_process', on_enter=['request_process_data_job']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='unload_batch', on_enter=['request_unload_batch']),
            State(name='move_sample_to_LCMS', on_enter=['request_move_sample_to_LCMS']),
            State(name='LCMS_process', on_enter=['request_LCMS_process']), #TODO confirm
            State(name='update_batch_index', on_enter=['request_batch_index_update']), #to check and remove
            State(name='final_state', on_enter='finalize_batch_processing')]

            
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'station_process', 'conditions':'is_station_job_ready'}, #'conditions':'all_batches_assigned'
            {'trigger':self._trigger_function,'source':'station_process','dest':'disable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'unload_batch', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'unload_batch','dest':'move_sample_to_LCMS', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'move_sample_to_LCMS','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'move_sample_to_LCMS','dest':'LCMS_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'LCMS_process','dest':'station_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'station_process','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)
        
    def request_unload_batch(self):
        robot_job = KukaLBRTask.from_args(name='UnloadEasySampler',params=[False,self._status['batch_index']+1],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        #current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job)

    def request_LCMS_process(self):
        pass

    def request_disable_auto_functions(self):
        #self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))
        pass

    def request_enable_auto_functions(self):
        #self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))
        pass
    
    def request_move_sample_to_LCMS(self):
        self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO change the node id and graph id for LCMS

    
    def request_batch_index_update(self):
        self._status['batch_index'] += 1
        self._status['batches_count'] += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self._status['batches_count'] = 0
        self._status['loaded_samples'] = 0
        self.to_init_state()
        
    def request_process_data_job(self):
        self._station.assign_station_op(OptimaxTempStirringOpDescriptor.from_args())

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()


