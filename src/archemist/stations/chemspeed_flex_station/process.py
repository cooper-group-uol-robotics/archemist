from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSCSVJobOpDescriptor, CSProcessingOpDescriptor
from archemist.core.util import Location
from archemist.core.state.station_process import StationProcess, StationProcessData

class ChemSpeedCSVProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door']),
            State(name='navigate_to_chemspeed', on_enter=['request_navigate_to_chemspeed']),
            State(name='retreat_from_chemspeed', on_enter=['request_navigate_to_chemspeed']),
            State(name='chemspeed_process', on_enter=['request_process_operation']),
            State(name='load_batch', on_enter=['request_load_batch']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='close_chemspeed_door', on_enter=['request_close_door']), 
            State(name='unload_batch', on_enter=['request_unload_batch']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'navigate_to_chemspeed'},
            {'source':'navigate_to_chemspeed','dest':'open_chemspeed_door', 'conditions':'are_req_robot_ops_completed'},
            {'source':'open_chemspeed_door','dest':'load_batch', 'unless':'is_station_operation_complete' ,'conditions':'are_req_station_ops_completed'},
            {'source':'load_batch','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update','dest':'load_batch', 'unless':'are_all_batches_loaded'},
            {'source':'added_batch_update','dest':'close_chemspeed_door', 'conditions':'are_all_batches_loaded'},
            {'source':'close_chemspeed_door','dest':'retreat_from_chemspeed', 'conditions':'are_req_station_ops_completed'},            
            {'source':'retreat_from_chemspeed','dest':'chemspeed_process', 'unless':'is_station_operation_complete', 'conditions':'are_req_robot_ops_completed'},
            {'source':'chemspeed_process','dest':'navigate_to_chemspeed', 'conditions':'are_req_station_ops_completed', 'before':'process_batches'},
            {'source':'open_chemspeed_door','dest':'unload_batch', 'conditions':['is_station_operation_complete','are_req_station_ops_completed']},
            {'source':'unload_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'unload_batch', 'unless':'are_all_batches_unloaded', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'close_chemspeed_door', 'conditions':'are_all_batches_unloaded'},
            {'source':'retreat_from_chemspeed','dest':'final_state', 'conditions':['are_req_robot_ops_completed','is_station_operation_complete']}
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0
        self._process_data.status['operation_complete'] = False

    def request_open_door(self):
        station_op = CSOpenDoorOpDescriptor.from_args()
        self.request_station_op(station_op)

    def request_close_door(self):
        station_op = CSCloseDoorOpDescriptor.from_args()
        self.request_station_op(station_op)

    def request_load_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='LoadChemSpeed',params=[True,batch_index+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def request_unload_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='UnloadChemSpeed',params=[False,batch_index],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index - 1].id
        self.request_robot_op(robot_op,current_batch_id)

    def request_navigate_to_chemspeed(self):
        robot_op = KukaNAVTask.from_args(Location(26,1,''), False) #TODO get this property from the config
        self.request_robot_op(robot_op)

    def request_process_operation(self):
        current_op = self._process_data.batches[-1].recipe.get_current_task_op()
        if isinstance (current_op,CSCSVJobOpDescriptor):
            combined_dispense_info = {k: [] for k in self._station.used_liquids_names}
            for batch in self._process_data.batches:
                current_op = batch.recipe.get_current_task_op()
                current_op_dispense_info = current_op.dispense_info
                for liquid, dispense_vals in current_op_dispense_info.items():
                    combined_dispense_info[liquid] += dispense_vals
            combined_op = CSCSVJobOpDescriptor.from_args(dispense_info=combined_dispense_info)
            self.request_station_op(combined_op)
        elif isinstance(current_op, CSProcessingOpDescriptor):
            self.request_station_op(current_op)

    def update_loaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)
        self._process_data.status['batch_index'] += 1

    def update_unloaded_batch(self):
        self._process_data.status['batch_index'] -= 1
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()

    ''' transition callbacks '''

    def process_batches(self):
        last_complete_station_op_uuid = self._process_data.station_ops_history[-1]
        req_operation_op = self._station.completed_station_ops[last_complete_station_op_uuid]
        for batch in self._process_data.batches:
            for i in range(0, batch.num_samples):
                dispense_info = {k: [v[i]] for k,v in 
                                batch.recipe.get_current_task_op().dispense_info.items()}
                sample_op = CSCSVJobOpDescriptor.from_args(dispense_info=dispense_info)
                sample_op.copy_stamps(req_operation_op)
                batch.add_station_op_to_current_sample(sample_op)
                batch.process_current_sample()
        self._process_data.status['operation_complete'] = True

    def are_all_batches_loaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)
    
    def are_all_batches_unloaded(self):
        return self._process_data.status['batch_index'] == 0

    def is_station_operation_complete(self):
            return self._process_data.status['operation_complete']

