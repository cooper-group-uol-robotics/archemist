from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from .state import SampleColorOpDescriptor
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.state.station_process import StationProcess, StationProcessData

class LightBoxProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='load_sample', on_enter=['request_load_sample_job']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='unload_sample', on_enter=['request_unload_sample_job']),
            State(name='station_process', on_enter=['request_process_data_job']),
            State(name='update_batch_index', on_enter=['request_batch_index_update']),
            State(name='update_sample_index', on_enter=['request_sample_index_update']),
            State(name='final_state', on_enter='finalize_batch_processing')]            

        ''' Transitions '''
        transitions = [
            { 'source':'init_state', 'dest': 'prep_state'},
            { 'source':'prep_state','dest':'disable_auto_functions'},
            { 'source':'disable_auto_functions','dest':'load_sample', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'load_sample','dest':'station_process', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'station_process','dest':'unload_sample', 'conditions':'are_req_station_ops_completed', 'before':'process_sample'},
            { 'source':'unload_sample','dest':'update_sample_index', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'update_sample_index','dest':'load_sample', 'unless':'are_all_samples_loaded'},
            { 'source':'update_sample_index','dest':'update_batch_index', 'conditions':'are_all_samples_loaded'},
            { 'source':'update_batch_index','dest':'load_sample', 'unless':'are_all_batches_processed'},
            { 'source':'update_batch_index','dest':'enable_auto_functions', 'conditions':'are_all_batches_processed'},
            { 'source':'enable_auto_functions','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]
        super().__init__(station, process_data, states, transitions)
        
    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0
        self._process_data.status['sample_index'] = 0

    def request_disable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def request_enable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def request_load_sample_job(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        perform_6p = False # this will be later evaluated by the KMRiiwa handler
        allow_auto_func = False # to stop auto charing and calibration when presentig the sample
        robot_job = KukaLBRTask.from_args(name='PresentVial',
                                          params=[perform_6p,batch_index+1, sample_index+1,allow_auto_func],
                                        type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def request_unload_sample_job(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        perform_6p = False # this will be later evaluated by the KMRiiwa handler
        allow_auto_func = False
        robot_job = KukaLBRTask.from_args(name='ReturnVial',
                                          params=[perform_6p,batch_index+1, sample_index+1,allow_auto_func],
                                        type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def request_process_data_job(self):
        station_op = SampleColorOpDescriptor.from_args()
        self.request_station_op(station_op)

    def request_sample_index_update(self):
        self._process_data.status['sample_index'] += 1

    def request_batch_index_update(self):
        self._process_data.status['batch_index'] += 1
        self._process_data.status['sample_index'] = 0

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()

    ''' transition callbacks '''

    def are_all_samples_loaded(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        return sample_index == self._process_data.batches[batch_index].num_samples

    def are_all_batches_processed(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)

    def process_sample(self):
        batch_index = self._process_data.status['batch_index']
        last_operation_op_uuid = self._process_data.station_ops_history[-1]
        last_operation_op = self._station.completed_station_ops[last_operation_op_uuid]
        current_batch = self._process_data.batches[batch_index]
        current_batch.add_station_op_to_current_sample(last_operation_op)
        current_batch.process_current_sample()


