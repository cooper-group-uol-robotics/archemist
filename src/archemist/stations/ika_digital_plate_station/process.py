from transitions import State
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.robot import RobotTaskType
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.state.station import Station
from typing import Dict

from archemist.stations.ika_digital_plate_station.state import IKAStirringOpDescriptor


class IKAStirPlateSm(StationProcessFSM):
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)

        ''' States '''
        states = [ State(name='init_state'),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='place_8_well_rack', on_enter=['request_8_well_rack']),
            State(name='place_pxrd_rack', on_enter=['request_pxrd_rack']),
            State(name='load_stir_plate', on_enter=['request_load_stir_plate']),
            State(name='stir', on_enter=['request_stir_op']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions', 'dest': 'place_8_well_rack', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function, 'source':'place_8_well_rack', 'dest': 'place_pxrd_rack', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'place_pxrd_rack', 'dest': 'load_stir_plate', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'load_stir_plate','dest':'stir', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'stir','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def request_8_well_rack(self):
        robot_job = KukaLBRTask.from_args(name='LoadEightWRackYumiStation',params=[True], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_pxrd_rack(self):
        robot_job = KukaLBRTask.from_args(name='LoadPXRDRackYumiStation',params=[False], 
                                            type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_load_stir_plate(self):
        robot_job = YuMiRobotTask.from_args(name='loadIKAPlate', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_stir_op(self):
        current_op = self._station.assigned_batches[self._status['batch_index']].recipe.get_current_task_op()
        if isinstance(current_op, IKAStirringOpDescriptor):
            self._station.assign_station_op(current_op)
    
    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))
