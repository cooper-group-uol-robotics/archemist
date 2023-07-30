from transitions import State
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.robot import RobotTaskType
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.core.state.station_process import StationProcess
from archemist.core.state.station import Station, StationProcessData
from typing import Dict

from archemist.stations.ika_digital_plate_station.state import IKAStirringOpDescriptor


class CrystalBotWorkflowProcess(StationProcess):
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='place_8_well_rack', on_enter=['request_8_well_rack']),
            State(name='place_pxrd_rack', on_enter=['request_pxrd_rack']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='load_stir_plate', on_enter=['request_load_stir_plate']),
            State(name='stir', on_enter=['request_stir_op']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'place_8_well_rack'},
            {'source':'place_8_well_rack', 'dest': 'place_pxrd_rack', 'conditions':'are_req_robot_ops_completed'},
            {'source':'place_pxrd_rack', 'dest': 'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update', 'dest': 'load_stir_plate'},
            {'source':'load_stir_plate','dest':'stir', 'conditions':'are_req_robot_ops_completed'},
            {'source':'stir','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def request_8_well_rack(self):
        perform_6p = True
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='LoadEightWRackYumiStation',params=[perform_6p], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def request_pxrd_rack(self):
        perform_6p = False
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='LoadPXRDRackYumiStation',params=[perform_6p], 
                                            type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def request_load_stir_plate(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = YuMiRobotTask.from_args(name='loadIKAPlate', location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_loaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)

    def request_stir_op(self):
        batch_index = self._process_data.status['batch_index']
        current_op = self._process_data.batches[batch_index].recipe.get_current_task_op()
        self.request_station_op(current_op)
    
    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
