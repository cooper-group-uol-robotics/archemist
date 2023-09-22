from transitions import State
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.state.robot import RobotTaskType
from archemist.core.state.station_process import StationProcess, StationProcessData
from archemist.core.state.station import Station

class YumiShakerPlateProcess(StationProcess):
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='load_shaker_plate', on_enter=['request_load_shaker_plate']),
            State(name='shake', on_enter=['request_shake_op']),
            State(name='unload_shaker_plate', on_enter=['request_unload_shaker_plate']),
            State(name='unscrew_caps', on_enter=['request_unscrew_caps']),
            State(name='pick_pxrd_rack', on_enter=['request_pxrd_rack_pickup']),
            State(name='pick_eightw_rack', on_enter=['request_eightw_rack_pickup']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'load_shaker_plate'},
            {'source':'load_shaker_plate','dest':'shake', 'conditions':'are_req_robot_ops_completed'},
            {'source':'shake','dest':'unload_shaker_plate', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_shaker_plate','dest':'unscrew_caps', 'conditions':'are_req_robot_ops_completed'},
            {'source':'unscrew_caps','dest':'pick_pxrd_rack', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pick_pxrd_rack','dest':'pick_eightw_rack', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pick_eightw_rack','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'final_state'},
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def request_load_shaker_plate(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = YuMiRobotTask.from_args(name='invertAndLoadShakerPlate', location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def request_shake_op(self):
        batch_index = self._process_data.status['batch_index']
        current_op = self._process_data.batches[batch_index].recipe.get_current_task_op()
        self.request_station_op(current_op)

    def request_unload_shaker_plate(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = YuMiRobotTask.from_args(name='invertAndLoadWellPlate', location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def request_unscrew_caps(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = YuMiRobotTask.from_args(name='unscrewCaps', location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def request_pxrd_rack_pickup(self):
        perform_6p = True
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='UnloadPXRDRackYumiStation',params=[perform_6p], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)
        
    def request_eightw_rack_pickup(self):
        perform_6p = False
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='UnloadEightWRackYumiStation',params=[perform_6p], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_unloaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)

    def finalize_batch_processing(self):
        for batch in self._process_data.batches:
            self._station.process_assinged_batch(batch)


