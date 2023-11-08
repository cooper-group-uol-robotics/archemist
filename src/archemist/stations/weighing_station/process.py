from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from .state import WeighingStation, SampleWeighingOpDescriptor, BalanceCloseDoorOpDescriptor, BalanceOpenDoorOpDescriptor
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location
import time

class WeighingSM(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'loaded_samples' not in self._status.keys():
            self._status['loaded_samples'] = 0
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False

        ''' States '''
        states = [State(name='init_state'), 
            State(name='open_balance_door', on_enter=['request_open_door']),
            State(name='navigate_to_weighing_station', on_enter=['request_navigate_to_weighing']),
            State(name='load_sample', on_enter=['request_load_sample_job']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='unload_sample', on_enter=['request_unload_sample_job']),
            State(name='station_process', on_enter=['request_process_data_job']),
            State(name='close_balance_door', on_enter=['request_close_door']), 
            State(name='final_state', on_enter='finalize_batch_processing')]
            
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions','dest':'navigate_to_weighing_station', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'navigate_to_weighing_station','dest':'open_balance_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'open_balance_door','dest':'load_sample', 'unless':'is_station_operation_complete', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'load_sample','dest':'close_balance_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'close_balance_door','dest':'station_process', 'unless':'is_station_operation_complete', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'station_process','dest':'open_balance_door', 'conditions':'is_station_job_ready','before':'process_sample'}, 
            {'trigger':self._trigger_function, 'source':'open_balance_door','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','is_station_operation_complete']}, 
            # {'trigger':self._trigger_function, 'source':'unload_sample','dest':'close_balance_door', 'conditions':['is_station_job_ready','is_funnel_unloaded']},
            # {'trigger':self._trigger_function, 'source':'close_balance_door','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','is_station_operation_complete']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready','is_station_operation_complete']}
        ]

        self.init_state_machine(states=states, transitions=transitions)
    
    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def is_funnel_loaded(self):
        return True
    
    def is_funnel_unloaded(self):
        return True
    
    def request_open_door(self):
        # self._station.assign_station_op(BalanceOpenDoorOpDescriptor.from_args())
        print('Door Operation is not enabled')
        

    def request_close_door(self):
         time.sleep(270)
        # self._station.assign_station_op(BalanceCloseDoorOpDescriptor.from_args())
         print('Door Operation is not enabled')


    def request_disable_auto_functions(self):
        # self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))
        
        pass
        

    def request_enable_auto_functions(self):
        #self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))
        pass

    def request_navigate_to_weighing(self):
        #self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config
        time.sleep(3)
        print('Robot is already in the location')

    def request_load_sample_job(self):
        # robot_job = KukaLBRTask.from_args(name='LoadWeighingStation',params=[True,self._status['batch_index']+1], 
        #                                     type=RobotTaskType.MANIPULATION, location=self._station.location)
        # current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        # self._station.request_robot_op(robot_job,current_batch_id)
        time.sleep(10)

    def request_unload_sample_job(self):
        # robot_job = KukaLBRTask.from_args(name='UnloadWeighingStation',params=[False,self._status['batch_index']+1],
        #                         type=RobotTaskType.MANIPULATION, location=self._station.location)
        # current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        # self._station.request_robot_op(robot_job,current_batch_id)
        time.sleep(3)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['operation_complete'] = False
        self.to_init_state()
        
    def request_process_data_job(self):
        op = SampleWeighingOpDescriptor.from_args()
        self._station.assign_station_op(op)
        self._status['operation_complete'] = True

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()


