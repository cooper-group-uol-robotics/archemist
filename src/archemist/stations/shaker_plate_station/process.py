from transitions import State
from archemist.core.persistence.object_factory import StationFactory
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.state.robot import RobotTaskType
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.state.station import Station
from typing import Dict

from archemist.stations.shaker_plate_station.state import ShakeOpDescriptor

class YumiShakerPlateSm(StationProcessFSM):
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        self.operation_complete = False
        self._current_batch_index = 0

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='load_shaker_plate', on_enter=['request_load_shaker_plate', '_print_state']),
            State(name='shake', on_enter=['request_shake_op', '_print_state']),
            State(name='unload_shaker_plate', on_enter=['request_unload_shaker_plate', '_print_state']),
            State(name='unscrew_caps', on_enter=['request_unscrew_caps', '_print_state']),
            State(name='pick_pxrd_rack', on_enter=['request_pxrd_rack_pickup', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]

        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'load_shaker_plate', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'load_shaker_plate','dest':'shake', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'shake','dest':'unload_shaker_plate', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'unload_shaker_plate','dest':'unscrew_caps', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'unscrew_caps','dest':'pick_pxrd_rack', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pick_pxrd_rack','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def request_load_shaker_plate(self):
        robot_job = YuMiRobotTask.from_args(name='invertAndLoadShakerPlate', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_shake_op(self):
        current_op = self._station.assigned_batches[self._current_batch_index].recipe.get_current_task_op()
        if isinstance(current_op, ShakeOpDescriptor):
            self._station.assign_station_op(current_op)

    def request_unload_shaker_plate(self):
        robot_job = YuMiRobotTask.from_args(name='invertAndLoadWellPlate', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_unscrew_caps(self):
        robot_job = YuMiRobotTask.from_args(name='unscrewCaps', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_pxrd_rack_pickup(self):
        robot_job = KukaLBRTask.from_args(name='UnloadPXRDRackYumiStation',params=[True], 
                                            type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')

