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

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='load_shaker_plate', on_enter=['request_load_shaker_plate']),
            State(name='shake', on_enter=['request_shake_op']),
            State(name='unload_shaker_plate', on_enter=['request_unload_shaker_plate']),
            State(name='unscrew_caps', on_enter=['request_unscrew_caps']),
            State(name='pick_pxrd_rack', on_enter=['request_pxrd_rack_pickup']),
            State(name='pick_eightw_rack', on_enter=['request_eightw_rack_pickup']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function, 'source':'disable_auto_functions', 'dest': 'load_shaker_plate', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'load_shaker_plate','dest':'shake', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'shake','dest':'unload_shaker_plate', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'unload_shaker_plate','dest':'unscrew_caps', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'unscrew_caps','dest':'pick_pxrd_rack', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pick_pxrd_rack','dest':'pick_eightw_rack', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pick_eightw_rack','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def request_load_shaker_plate(self):
        robot_job = YuMiRobotTask.from_args(name='invertAndLoadShakerPlate', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_shake_op(self):
        current_op = self._station.assigned_batches[self._status['batch_index']].recipe.get_current_task_op()
        if isinstance(current_op, ShakeOpDescriptor):
            self._station.assign_station_op(current_op)

    def request_unload_shaker_plate(self):
        robot_job = YuMiRobotTask.from_args(name='invertAndLoadWellPlate', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_unscrew_caps(self):
        robot_job = YuMiRobotTask.from_args(name='unscrewCaps', location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_pxrd_rack_pickup(self):
        robot_job = KukaLBRTask.from_args(name='UnloadPXRDRackYumiStation',params=[True], 
                                            type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)
        
    def request_eightw_rack_pickup(self):
        robot_job = KukaLBRTask.from_args(name='UnloadEightWRackYumiStation',params=[False], 
                                            type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self.to_init_state()

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))


