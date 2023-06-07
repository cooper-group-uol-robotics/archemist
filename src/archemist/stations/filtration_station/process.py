
from typing import Dict
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import FiltrationStation, FiltrationValveOpenOpDescriptor, FiltrationValveCloseOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location

class FiltrationStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False
        self._status['filtration_complete'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='navigate_to_filtration_station', on_enter=['request_navigate_to_filtration']),
            State(name='open_drain_valve', on_enter=['request_open_drain_valve']),
            State(name='filtration_process', on_enter=['request_filtration_process']),
            State(name='close_drain_valve', on_enter=['request_close_drain_valve']),
            State(name='pickup_funnel', on_enter=['request_pickup_funnel']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'open_drain_valve', 'conditions':['is_station_job_ready', 'all_batches_assigned']},
            {'trigger':self._trigger_function, 'source':'open_drain_valve','dest':'filtration_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'filtration_process','dest':'close_drain_valve', 'conditions':['is_station_job_ready', 'is_filtration_process_complete']},
            {'trigger':self._trigger_function, 'source':'close_drain_valve','dest':'disable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions','dest':'navigate_to_filtration_station', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'navigate_to_filtration_station','dest':'pickup_funnel', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pickup_funnel','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready', 'is_station_operation_complete']}
        ]   

        self.init_state_machine(states=states, transitions=transitions)

    def is_station_operation_complete(self):
        return self._status['operation_complete']
    
    def request_filtration_process(self):
        print('filteration_process')
        # Add timer
        # target_duration = kwargs['duration']
        # start_time = rospy.get_time()
        # elepsed_duration = 0
        # while elepsed_duration < target_duration:
        #     rospy.sleep(1)
        #     elepsed_duration = rospy.get_time() - start_time
        self._status['filtration_complete'] = True


    def request_open_drain_valve(self):
        self._station.assign_station_op(FiltrationValveOpenOpDescriptor.from_args())


    def request_close_drain_valve(self):
        self._station.assign_station_op(FiltrationValveCloseOpDescriptor.from_args())

    def is_filtration_process_complete(self) -> bool:
        return self._status['filtration_complete']

    def request_pickup_funnel(self):
        robot_job = KukaLBRTask.from_args(name='UnloadFiltrationStation',params=[False,self._status['batch_index']+1],
                                type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)
        self._status['operation_complete'] = True

    def request_navigate_to_filtration(self):
        self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config


    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

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



