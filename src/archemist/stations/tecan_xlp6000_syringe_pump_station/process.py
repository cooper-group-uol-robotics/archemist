
from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import SyringePump, SyringePumpDispenseOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location


class SyringePumpStationSm(StationProcessFSM):

    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False
        self._status['pump_capacity'] = 25
        #self._status['split_volume'] = []
        self._status['prep_done'] = False
        #self._status['iterations'] = 0
        #self._status['iterations_done'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='prep_state', on_enter=['preperation']), 
            State(name='dispense', on_enter=['request_dispense_operation']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'prep_state', 'conditions':['is_station_job_ready', 'all_batches_assigned']},
            {'trigger':self._trigger_function, 'source':'prep_state', 'dest': 'dispense', 'conditions':['is_station_job_ready', 'is_prep_done']},
            {'trigger':self._trigger_function, 'source':'dispense','dest':'final_state', 'conditions':['is_station_job_ready','is_station_operation_complete'], 'before':'process_sample'}
        ]   

        self.init_state_machine(states=states, transitions=transitions)
   
    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_dispense_operation(self):
        #self._current_batch_pump_info['volume'] = self._status['split_volume'][self._status['iterations']]
        op = SyringePumpDispenseOpDescriptor.from_args(pump_info = self._current_batch_pump_info)
        self._station.assign_station_op(op)
        #self._status['iterations'] += 1
        #if self._status['iterations'] == len(self._status['split_volume']):
        self._status['operation_complete'] = True
             
    def is_prep_done(self):
        print('preperation_done')
        return self._status['prep_done']
    
    def preperation(self):
        current_op = self._station.assigned_batches[0].recipe.get_current_task_op()
        self._current_batch_pump_info = {}
        self._current_batch_pump_info['withdraw_port'] = int(current_op.withdraw_port)
        self._current_batch_pump_info['dispense_port'] = int(current_op.dispense_port)
        self._current_batch_pump_info['speed'] = int(current_op.speed) 
        self._current_batch_pump_info['volume'] = int(current_op.volume)
        self._status['prep_done'] = True

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['operation_complete'] = False
        #self._status['spliting_done'] = False
        #self._status['iterations'] = 0
        self.to_init_state()

    
    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()


