
from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import FiltrationStation, BaseValveOpenOpDescriptor, BaseValveCloseOpDescriptor, VacuumOpenOpDescriptor, VacuumCloseOpDescriptor, DrainValveOpenOpDescriptor, DrainValveCloseOpDescriptor, IdleOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location
import time

class FiltrationStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False
        self._status['filtration_complete'] = False
        self._status['funnel_filled'] = False
        self._status['optimax_empty'] = True
        self._status['draining_complete'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='navigate_to_filtration_station', on_enter=['request_navigate_to_filtration']),
            State(name='open_base_valve', on_enter=['request_open_base_valve']),
            State(name='close_base_valve', on_enter=['request_close_base_valve']),
            State(name='open_vacuum', on_enter=['request_open_vacuum']),
            State(name='close_vacuum', on_enter=['request_close_vacuum']),
            State(name='open_drain_valve', on_enter=['request_open_draining']),
            State(name='close_drain_valve', on_enter=['request_close_draining']),
            State(name='stop_all_process', on_enter=['request_stop_all']),
            State(name='pickup_funnel', on_enter=['request_pickup_funnel']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'open_vacuum', 'conditions':['is_station_job_ready', 'all_batches_assigned']},
            {'trigger':self._trigger_function, 'source':'open_vacuum', 'dest': 'open_base_valve', 'conditions':['is_station_job_ready', 'all_batches_assigned']},
            {'trigger':self._trigger_function, 'source':'open_base_valve','dest':'close_base_valve', 'conditions':['is_station_job_ready','is_funnel_filled']},
            {'trigger':self._trigger_function, 'source':'close_base_valve','dest':'close_vacuum', 'conditions':['is_station_job_ready', 'is_filtration_process_complete'], 'before':'process_sample'},
            {'trigger':self._trigger_function, 'source':'close_vacuum','dest':'open_vacuum', 'unless':'is_optimax_empty' , 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'close_vacuum','dest':'open_drain_valve', 'conditions':['is_station_job_ready', 'is_optimax_empty']},
            {'trigger':self._trigger_function, 'source':'open_drain_valve','dest':'close_drain_valve', 'conditions':['is_station_job_ready','is_draining_complete']},
            {'trigger':self._trigger_function, 'source':'close_drain_valve','dest':'disable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions','dest':'navigate_to_filtration_station', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'navigate_to_filtration_station','dest':'pickup_funnel', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pickup_funnel','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready', 'is_station_operation_complete']}
        ]   

        self.init_state_machine(states=states, transitions=transitions)

    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_open_base_valve(self):
        self._station.assign_station_op(BaseValveOpenOpDescriptor.from_args())
        self.timer(20)
        self._status['funnel_filled'] = True

    def request_close_base_valve(self):
        self._station.assign_station_op(BaseValveCloseOpDescriptor.from_args())

    def request_open_vacuum(self):
        self._station.assign_station_op(VacuumOpenOpDescriptor.from_args())
        

    def request_close_vacuum(self):
        self._station.assign_station_op(VacuumCloseOpDescriptor.from_args())

    def request_open_draining(self):
        self._station.assign_station_op(DrainValveOpenOpDescriptor.from_args())


    def request_close_draining(self):
        self._station.assign_station_op(DrainValveCloseOpDescriptor.from_args())

    def request_stop_all(self):
        self._station.assign_station_op(IdleOpDescriptor.from_args())

    def is_filtration_process_complete(self) -> bool:
        self.timer(180)
        self._status['filtration_complete'] = True
        return self._status['filtration_complete']
    
    def is_funnel_filled(self) -> bool:
        self.timer(60)
        self._status['draining_complete'] = True
        return self._status['funnel_filled']

    def is_optimax_empty(self) -> bool:
        return self._status['optimax_empty']

    def is_draining_complete(self) -> bool:
        return self._status['draining_complete']
    
    def request_pickup_funnel(self):
        # robot_job = KukaLBRTask.from_args(name='UnloadFiltrationStation',params=[False,self._status['batch_index']+1],
        #                         type=RobotTaskType.MANIPULATION, location=self._station.location)
        # current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        # self._station.request_robot_op(robot_job,current_batch_id)
        print("pickup funnel")
        self._status['operation_complete'] = True

    def request_navigate_to_filtration(self):
        # self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config
        print("kuka nav to filtration station")

    def request_disable_auto_functions(self):
        # self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))
        print("kuka disable auto")

    def request_enable_auto_functions(self):
        # self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))
        print("kuka enable auto")

    def request_process_operation(self):
        current_op = self._station.assigned_batches[-1].recipe.get_current_task_op()
        self._station.assign_station_op(current_op)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['filteration_complete'] = False
        self._status['operation_complete'] = False
        self._status['batch_index'] = 0
        self._status['batches_count'] = 0
        self.to_init_state()

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()

    def timer(self, seconds):
        start_time = time.time()
        end_time = start_time + seconds
        
        while time.time() < end_time:
            remaining_time = int(end_time - time.time())
            print(f"Time remaining: {remaining_time} seconds", end="\r", flush=True)
            time.sleep(1)
        
