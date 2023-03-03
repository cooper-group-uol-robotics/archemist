from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.processing.station_process_fsm import StationProcessFSM

class OutputStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='place_batch', on_enter=['request_place_batch', '_print_state']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions', '_print_state']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions', '_print_state']),
            State(name='added_batch_update', on_enter=['update_loaded_batch', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'place_batch', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'place_batch','dest':'added_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'added_batch_update','dest':'place_batch', 'unless':'are_all_batches_loaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','are_all_batches_loaded']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def request_place_batch(self):
        robot_job = KukaLBRTask.from_args(name='PlaceRack',params=[False,self._current_batch_index+1],
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)
    
    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._current_batch_index = 0
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



