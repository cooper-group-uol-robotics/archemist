from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType, RobotTaskOpDescriptor
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.util import Location
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.state_machines.base_sm import BaseSm

class StationLoadingSm(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)

        self._currently_loaded_samples = 0
        self.operation_complete = False
        self.batch_mode = params_dict['batch_mode']
        self._batch_load_task = params_dict['batch_load_task']
        self._batch_unload_task = params_dict['batch_unload_task']
        self._sample_load_task = params_dict['sample_load_task']
        self._sample_unload_task = params_dict['sample_unload_task']

        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='load_sample', on_enter=['request_load_sample_job', '_print_state']),
            State(name='added_sample_update', on_enter=['update_sample_addition', '_print_state']),
            State(name='load_batch', on_enter=['request_load_batch', '_print_state']),
            State(name='added_batch_update', on_enter=['update_batch_loc_to_station', '_print_state']),
            State(name='station_process', on_enter=['request_operation', '_print_state']),
            State(name='unload_sample', on_enter=['request_unload_sample_job','_print_state']),
            State(name='removed_sample_update', on_enter=['update_sample_removal', '_print_state']),
            State(name='unload_batch', on_enter=['request_unload_batch_job','_print_state']),
            State(name='removed_batch_update', on_enter=['update_batch_removal', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_batch', conditions='all_batches_assigned')
        self.machine.add_transition('process_state_transitions',source='load_batch',dest='added_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions',source='added_batch_update',dest='load_sample', conditions='is_station_job_ready')
        

        if self.batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='added_sample_update', conditions='is_station_job_ready')
            self.machine.add_transition('process_state_transitions', source='added_sample_update',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='added_sample_update',dest='station_process', conditions=['are_all_samples_loaded','is_station_job_ready'])

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample',conditions='is_station_job_ready', before='process_batch')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='removed_sample_update', conditions='is_station_job_ready')
            self.machine.add_transition('process_state_transitions', source='removed_sample_update',dest='unload_sample', conditions='is_station_job_ready', unless='are_all_samples_unloaded')
            self.machine.add_transition('process_state_transitions', source='removed_sample_update',dest='unload_batch', conditions=['are_all_samples_unloaded','is_station_job_ready'])

        else:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='is_station_job_ready')
            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_job_ready', before='process_sample')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='added_sample_update', conditions='is_station_job_ready', unless='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='added_sample_update',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded')

            self.machine.add_transition('process_state_transitions', source='added_sample_update',dest='unload_batch', conditions=['are_all_samples_loaded','is_station_job_ready'] , before='reset_samples')
            
        self.machine.add_transition('process_state_transitions',source='unload_batch',dest='removed_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions',source='removed_batch_update',dest='load_batch', unless='are_all_batches_processed', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions',source='removed_batch_update',dest='final_state', conditions=['is_station_job_ready','are_all_batches_processed'], before='reset_batches')

    def is_station_job_ready(self):
        return not self._station.has_assigned_station_op() and not self._station.has_requested_robot_op()

    def are_all_samples_loaded(self):
        return self._currently_loaded_samples == self._station.assigned_batches[self._current_batch_index].num_samples

    def are_all_samples_unloaded(self):
        return self._currently_loaded_samples == 0

    def are_all_batches_processed(self):
        return self._current_batches_count == self._station.batch_capacity

    def update_batch_removal(self):
        self.update_batch_loc_to_robot()
        self._current_batches_count += 1
        if self._current_batches_count == self._station.batch_capacity:
            self._current_batch_index = 0
        else:
            self._current_batch_index += 1

    def reset_samples(self):
        self._currently_loaded_samples = 0

    def reset_batches(self):
        self._current_batch_index = 0
        self._current_batches_count = 0

    def request_load_sample_job(self):
        sample_index = self._currently_loaded_samples
        robot_job = (RobotTaskOpDescriptor.from_args(name=self._sample_load_task, params=[sample_index]))
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)


    def request_load_batch(self):
        robot_job = (KukaLBRTask.from_args(name=self._batch_load_task,params=[False,self._current_batch_index],
                                    type=RobotTaskType.UNLOAD_FROM_ROBOT,location=self._station.location))
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_batch_job(self):
        robot_job = (KukaLBRTask.from_args(name=self._batch_unload_task,params=[False,self._current_batch_index],
                                    type=RobotTaskType.UNLOAD_FROM_ROBOT,location=self._station.location))
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_operation(self):
        current_op_dict = self._station.assigned_batches[self._current_batch_index].recipe.get_current_task_op_dict()
        current_op = StationFactory.create_op_from_dict(current_op_dict)
        self._station.assign_station_op(current_op)

    def request_unload_sample_job(self):
        sample_index = self._currently_loaded_samples
        robot_job = (RobotTaskOpDescriptor.from_args(name=self._sample_unload_task, params=[sample_index]))
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def update_sample_addition(self):
        self._currently_loaded_samples += 1

    def update_sample_removal(self):
        self._currently_loaded_samples -= 1

    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        for _ in range(0, self._station.assigned_batches[self._current_batch_index].num_samples):
                self._station.assigned_batches[self._current_batch_index].add_station_op_to_current_sample(last_operation_op)
                self._station.assigned_batches[self._current_batch_index].process_current_sample()
        self.operation_complete = True

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._current_batch_index].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._current_batch_index].process_current_sample()
    
    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



