from transitions import State
from .state import PXRDStatus
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.state.station_process import StationProcess, StationProcessData
from archemist.core.util import Location

class PXRDProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='open_pxrd_door', on_enter=['request_open_pxrd_door']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']), 
            State(name='pxrd_process', on_enter=['request_pxrd_process']),
            State(name='load_pxrd', on_enter=['request_load_pxrd']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='close_pxrd_door', on_enter=['request_close_pxrd_door']), 
            State(name='unload_pxrd', on_enter=['request_unload_pxrd']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'disable_auto_functions'},
            {'source':'disable_auto_functions','dest':'open_pxrd_door', 'conditions':'are_req_robot_ops_completed'},
            {'source':'open_pxrd_door','dest':'load_pxrd', 'unless':'is_station_operation_complete' ,
                'conditions':'are_req_robot_ops_completed', 'before':'set_doors_to_open'},
            {'source':'open_pxrd_door','dest':'unload_pxrd', 'conditions':['is_station_operation_complete','are_req_robot_ops_completed']
                ,'before':'set_doors_to_open'},
            {'source':'load_pxrd','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update','dest':'close_pxrd_door'},
            {'source':'close_pxrd_door','dest':'enable_auto_functions', 'conditions':'are_req_robot_ops_completed',
                'before':'set_doors_to_closed'},
            {'source':'enable_auto_functions','dest':'pxrd_process', 'unless':'is_station_operation_complete', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pxrd_process','dest':'disable_auto_functions', 'conditions':'are_req_station_ops_completed', 'before':'process_batches'},
            {'source':'unload_pxrd','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'close_pxrd_door'},
            {'source':'enable_auto_functions','dest':'final_state', 'conditions':['are_req_robot_ops_completed','is_station_operation_complete']}
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''
    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0
        self._process_data.status['operation_complete'] = False

    def request_open_pxrd_door(self):
        door_loc = Location(node_id=19, graph_id=1)
        robot_op = KukaLBRTask.from_args(name='OpenDoors',params=[True], 
                                            type=RobotTaskType.MANIPULATION, location=door_loc)
        self.request_robot_op(robot_op)

    def request_close_pxrd_door(self):
        door_loc = Location(node_id=19, graph_id=1)
        robot_op = KukaLBRTask.from_args(name='CloseDoors',params=[True], 
                                            type=RobotTaskType.MANIPULATION, location=door_loc)
        self.request_robot_op(robot_op)

    def request_load_pxrd(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='LoadPXRD',params=[True], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_loaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)

    def request_unload_pxrd(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='UnloadPXRD',params=[True],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_unloaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)

    def request_disable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def request_enable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def request_pxrd_process(self):
        current_op = self._process_data.batches[-1].recipe.get_current_task_op()
        self.request_station_op(current_op)

    ''' transition callbacks'''
    def is_station_operation_complete(self):
        return self._process_data.status['operation_complete']

    def set_doors_to_open(self):
        self._station.status = PXRDStatus.DOORS_OPEN

    def set_doors_to_closed(self):
        self._station.status = PXRDStatus.DOORS_CLOSED

    def process_batches(self):
        last_complete_station_op_uuid = self._process_data.station_ops_history[-1]
        req_operation_op = self._station.completed_station_ops[last_complete_station_op_uuid]
        for batch in self._process_data.batches:
            for _ in range(0, batch.num_samples):
                batch.add_station_op_to_current_sample(req_operation_op)
                batch.process_current_sample()
        self._process_data.status['operation_complete'] = True

    def finalize_batch_processing(self):
        for batch in self._process_data.batches:
            self._station.process_assinged_batch(batch)


