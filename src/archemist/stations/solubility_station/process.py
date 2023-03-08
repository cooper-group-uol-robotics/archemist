from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.panda_robot.state import PandaRobotTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from .state import SolubilityOpDescriptor


class SolubilityStationSM(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'loaded_samples' not in self._status.keys():
            self._status['loaded_samples'] = 0

        ''' States '''
        states = [State(name='init_state'), 
            State(name='load_sample', on_enter=['request_load_sample_job']),
            State(name='unload_sample', on_enter=['request_unload_sample_job']),
            State(name='station_process', on_enter=['request_process_data_job']),
            State(name='update_batch_index', on_enter=['request_batch_index_update']),
            State(name='final_state', on_enter='finalize_batch_processing')]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'load_sample', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function, 'source':'load_sample','dest':'station_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'station_process','dest':'unload_sample', 'conditions':'is_station_job_ready', 'before':'process_sample'},
            {'trigger':self._trigger_function, 'source':'unload_sample','dest':'load_sample', 'conditions':'is_station_job_ready', 'unless':'are_all_samples_loaded'},
            {'trigger':self._trigger_function, 'source':'unload_sample','dest':'update_batch_index', 'conditions':['are_all_samples_loaded','is_station_job_ready'], 'before':'reset_station'},
            {'trigger':self._trigger_function, 'source':'update_batch_index','dest':'load_sample', 'conditions':'is_station_job_ready', 'unless':'are_all_batches_processed'},
            {'trigger':self._trigger_function, 'source':'update_batch_index','dest':'final_state', 'conditions':['are_all_batches_processed','is_station_job_ready']}
        ]

        self.init_state_machine(states=states, transitions=transitions)
        

    def are_all_samples_loaded(self):
        return self._status['loaded_samples'] == self._station.assigned_batches[self._status['batch_index']].num_samples

    def are_all_batches_processed(self):
        return self._status['batches_count'] == self._station.batch_capacity


    def reset_station(self):
        self._status['loaded_samples'] = 0

    def request_load_sample_job(self):
        self._status['loaded_samples'] += 1
        sample_index = self._status['loaded_samples']
        robot_job = PandaRobotTask.from_args(name='PresentVial',params=[self._status['batch_index']+1, sample_index],
                                        type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_sample_job(self):
        sample_index = self._status['loaded_samples']
        robot_job = PandaRobotTask.from_args(name='ReturnVial',params=[self._status['batch_index']+1, sample_index],
                                            type=RobotTaskType.MANIPULATION, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_batch_index_update(self):
        self._status['batch_index'] += 1
        self._status['batches_count'] += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self._status['batches_count'] = 0
        self._status['loaded_samples'] = 0
        self.to_init_state()
        
    def request_process_data_job(self):
        self._station.assign_station_op(SolubilityOpDescriptor.from_args())

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()

