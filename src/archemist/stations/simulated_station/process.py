from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType, RobotTaskOpDescriptor
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.state.station_process import StationProcess, StationProcessData
class SimulatedStationProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        self.batch_mode = kwargs['batch_mode']
        self.batch_load_task = kwargs['batch_load_task']
        self.batch_unload_task = kwargs['batch_unload_task']
        self.sample_load_task = kwargs['sample_load_task']
        self.sample_unload_task = kwargs['sample_unload_task']

        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='load_sample', on_enter=['request_load_sample']),
            State(name='advance_sample_index', on_enter=['update_sample_index']),
            State(name='load_batch', on_enter=['request_load_batch']),
            State(name='added_batch_update', on_enter=['update_batch_addition']),
            State(name='station_process', on_enter=['request_operation']),
            State(name='unload_sample', on_enter=['request_unload_sample']),
            State(name='removed_sample_update', on_enter=['update_sample_removal']),
            State(name='unload_batch', on_enter=['request_unload_batch']),
            State(name='removed_batch_update', on_enter=['update_batch_removal']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'load_batch'},
            {'source':'load_batch','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update','dest':'load_sample'}]
        
        if self._process_data.status['batch_mode']:
            transitions += [{'source':'load_sample','dest':'advance_sample_index', 'conditions':'are_req_robot_ops_completed'},
                {'source':'advance_sample_index','dest':'load_sample', 'unless':'are_all_samples_loaded'},
                {'source':'advance_sample_index','dest':'station_process', 'conditions':'are_all_samples_loaded'},
                {'source':'station_process','dest':'unload_sample','conditions':'are_req_station_ops_completed', 'before':'process_batch'},
                {'source':'unload_sample','dest':'removed_sample_update', 'conditions':'are_req_robot_ops_completed'},
                {'source':'removed_sample_update','dest':'unload_sample', 'unless':'are_all_samples_unloaded'},
                {'source':'removed_sample_update','dest':'unload_batch', 'conditions':'are_all_samples_unloaded'},
                ]
        else:
            transitions += [{'source':'load_sample','dest':'station_process', 'conditions':'are_req_robot_ops_completed'},
                {'source':'station_process','dest':'unload_sample', 'conditions':'are_req_station_ops_completed', 'before':'process_sample'},
                {'source':'unload_sample','dest':'advance_sample_index', 'conditions':'are_req_robot_ops_completed'},
                {'source':'advance_sample_index','dest':'load_sample', 'unless':'are_all_samples_loaded'},
                {'source':'advance_sample_index','dest':'unload_batch', 'conditions':'are_all_samples_loaded'}]
        
        transitions += [{'source':'unload_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'load_batch', 'unless':'are_all_batches_processed'},
            {'source':'removed_batch_update','dest':'final_state', 'conditions':'are_all_batches_processed'}]
        
        super().__init__(station, process_data, states, transitions, **kwargs)
        
    ''' state callbacks '''
    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0
        self._process_data.status['sample_index'] = 0

    def request_load_sample(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        robot_job = (RobotTaskOpDescriptor.from_args(name=self.sample_load_task, params=[sample_index]))
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def update_sample_index(self):
        self._process_data.status['sample_index'] += 1

    def request_load_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_job = (RobotTaskOpDescriptor.from_args(name=self.batch_load_task, 
                                                        type=RobotTaskType.UNLOAD_FROM_ROBOT,
                                                        location=self._station.location, params=[batch_index]))
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def update_batch_addition(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)

    def request_operation(self):
        batch_index = self._process_data.status['batch_index']
        station_op = self._process_data.batches[batch_index].recipe.get_current_task_op()
        self.request_station_op(station_op)

    def request_unload_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_job = (RobotTaskOpDescriptor.from_args(name=self.batch_unload_task, 
                                                        type=RobotTaskType.LOAD_TO_ROBOT,
                                                        location=self._station.location, params=[batch_index]))
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def request_unload_sample(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        robot_job = (RobotTaskOpDescriptor.from_args(name=self.sample_unload_task, params=[sample_index]))
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def update_sample_removal(self):
        self._process_data.status['sample_index'] -= 1

    def update_batch_removal(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)
        self._process_data.status['batch_index'] += 1
        self._process_data.status['sample_index'] = 0

    def finalize_batch_processing(self):
        for batch in self._process_data.batches:
            self._station.process_assigned_batches()

    ''' transition callback '''
    def are_all_batches_processed(self):
        batch_index = self._process_data.status['batch_index']
        return batch_index == len(self._process_data.batches)

    def are_all_samples_loaded(self):
        sample_index = self._process_data.status['sample_index']
        batch_index = self._process_data.status['batch_index']
        return sample_index == self._process_data.batches[batch_index].num_samples

    def are_all_samples_unloaded(self):
        return self._status['loaded_samples'] == 0

    def are_all_batches_processed(self):
        return self._status['batches_count'] == self._station.batch_capacity

    def process_batch(self):
        batch_index = self._process_data.status['batch_index']
        last_operation_op_uuid = self._process_data.station_ops_history[-1]
        last_operation_op = self._station.completed_station_ops[last_operation_op_uuid]
        current_batch = self._process_data.batches[batch_index]
        for _ in range(0, current_batch.num_samples):
                current_batch.add_station_op_to_current_sample(last_operation_op)
                current_batch.process_current_sample()

    def process_sample(self):
        batch_index = self._process_data.status['batch_index']
        last_operation_op_uuid = self._process_data.station_ops_history[-1]
        last_operation_op = self._station.completed_station_ops[last_operation_op_uuid]
        self._process_data.batches[batch_index].add_station_op_to_current_sample(last_operation_op)
        self._process_data.batches[batch_index].process_current_sample()
