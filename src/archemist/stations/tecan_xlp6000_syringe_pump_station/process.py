
from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import SyringePump, SyringePumpDispenseOpDescriptor, SyringePumpWithdrawOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location


class SyringePumpStationSm(StationProcessFSM):

    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False
        self._status['withdraw_port'] = 4
        self._status['dispense_port'] = 7
        self._status['withdraw_speed'] = 150
        self._status['dispense_speed'] = 150
        self._status['total_volume'] = 30
        self._status['pump_capacity'] = 25
        self._status['split_volume'] = []
        self._status['spliting_done'] = False
        self._status['iterations'] = 0
        self._status['iterations_done'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='split_volume', on_enter=['request_split_volume']),
            State(name='withdraw', on_enter=['request_withdraw_operation']),
            State(name='dispense', on_enter=['request_dispense_operation']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'split_volume', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'split_volume', 'dest': 'withdraw', 'conditions':['is_station_job_ready','is_splitting_done']},
            {'trigger':self._trigger_function, 'source':'withdraw', 'dest': 'dispense', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'dispense', 'dest': 'withdraw', 'conditions':'is_station_job_ready','unless':'is_station_operation_complete'},
            {'trigger':self._trigger_function, 'source':'dispense','dest':'final_state', 'conditions':'is_station_operation_complete'}
        ]   

        self.init_state_machine(states=states, transitions=transitions)
   
    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_withdraw_operation(self):
        self._station.assign_station_op(SyringePumpWithdrawOpDescriptor)
        #self._station.assign_station_op(SyringePumpWithdrawOpDescriptor.from_args(self.current_op_dispense_info))

    def request_dispense_operation(self):
        self._station.assign_station_op(SyringePumpDispenseOpDescriptor)
        #self._station.assign_station_op(SyringePumpDispenseOpDescriptor.from_args(self.current_op_dispense_info))
        self._status['iterations'] += 1
        if self._status['iterations'] == len(self._status['split_volume']):
             self._status['operation_complete'] = True
             
    def is_splitting_done(self):
        print('splitting_done')
        return self._status['spliting_done']

    def request_split_volume(self):
        current_op = self._station.assigned_batches[-1].recipe.get_current_task_op()
        self.current_op_dispense_info = current_op.pump_info
        iterations, last_iteration_volume = divmod(self._status['total_volume'], self._status['pump_capacity'])
        for i in range(iterations):
            self._status['split_volume'].append(self._status['pump_capacity'])
        if last_iteration_volume is not 0:
            self._status['split_volume'].append(last_iteration_volume)
        self._status['spliting_done'] = True

    def finalize_batch_processing(self):
        self._status['operation_complete'] = False
        self._status['spliting_done'] = False
        self._status['iterations'] = 0
        self.to_init_state()

    