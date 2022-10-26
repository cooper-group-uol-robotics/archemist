from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.stations.light_box_station import SampleColorOpDescriptor
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.processing.state_machines.base_sm import BaseSm

class LightBoxSM(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        self._currently_loaded_samples = 0
        self.operation_complete = False

        ''' States '''
        states = [State(name='init_state', on_enter='_print_state'), 
            State(name='load_sample', on_enter=['request_load_sample_job','_print_state']),
            State(name='unload_sample', on_enter=['request_unload_sample_job','_print_state']),
            State(name='station_process', on_enter=['request_process_data_job', '_print_state']),
            State(name='update_batch_index', on_enter=['request_batch_index_update', '_print_state']),
            State(name='final_state', on_enter='finalize_batch_processing')]
            
            
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_sample', conditions='all_batches_assigned')

        # load_sample transitions
        self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='is_station_job_ready')

        # station_process transitions
        self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_job_ready', before='process_sample')

        # unload_sample transitions
        self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded')
        self.machine.add_transition('process_state_transitions', source='unload_sample',dest='update_batch_index', conditions=['are_all_samples_loaded','is_station_job_ready'], before='reset_station')
        
        self.machine.add_transition('process_state_transitions', source='update_batch_index',dest='load_sample', conditions='is_station_job_ready', unless='are_all_batches_processed')

        self.machine.add_transition('process_state_transitions', source='update_batch_index',dest='final_state', conditions=['are_all_batches_processed','is_station_job_ready'])


    def are_all_samples_loaded(self):
        return self._currently_loaded_samples == self._station.assigned_batches[self._current_batch_index].num_samples

    def are_all_batches_processed(self):
        return self._current_batches_count == self._station.batch_capacity


    def reset_station(self):
        self._currently_loaded_samples = 0

    def request_load_sample_job(self):
        self._currently_loaded_samples += 1
        sample_index = self._currently_loaded_samples
        perform_6p = False # this will be later evaluated by the KMRiiwa handler
        allow_auto_func = False # to stop auto charing and calibration when presentig the sample
        robot_job = KukaLBRTask.from_args(name='PresentVial',params=[perform_6p,self._current_batch_index+1,sample_index,allow_auto_func],
                                        type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_sample_job(self):
        sample_index = self._currently_loaded_samples
        perform_6p = False
        if self.are_all_samples_loaded() and (self._current_batches_count - 1) == self._station.batch_capacity:
            allow_auto_func = True # enable auto charge and calibration since we are done un/loading samples
        else:
            allow_auto_func = False
        robot_job = KukaLBRTask.from_args(name='ReturnVial',params=[perform_6p,self._current_batch_index+1,sample_index,allow_auto_func],
                                            type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_batch_index_update(self):
        self._current_batch_index += 1
        self._current_batches_count += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._current_batch_index = 0
        self._current_batches_count = 0
        self.to_init_state()
        
    def request_process_data_job(self):
        self._station.assign_station_op(SampleColorOpDescriptor.from_args())

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._current_batch_index].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._current_batch_index].process_current_sample()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')


