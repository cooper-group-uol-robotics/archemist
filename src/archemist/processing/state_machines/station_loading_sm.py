from typing import Dict
from transitions import Machine, State
from archemist.state.station import Station
from archemist.state.robot import RobotTaskType, RobotTaskOpDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.util import Location
from archemist.persistence.object_factory import StationFactory
from archemist.processing.state_machines.base_sm import BaseSm

class StationLoadingSm(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)

        self.operation_complete = False
        self.batch_mode = params_dict['batch_mode']
        self._rack_load_task = params_dict['rack_load_task']
        self._rack_unload_task = params_dict['rack_unload_task']
        self._vial_load_task = params_dict['vial_load_task']
        self._vial_unload_task = params_dict['vial_unload_task']

        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='load_sample', on_enter=['request_load_vial_job', '_print_state']),
            State(name='load_rack', on_enter=['request_load_rack_job', '_print_state']), 
            State(name='station_process', on_enter=['request_operation', '_print_state']),
            State(name='unload_sample', on_enter=['request_unload_vial_job','_print_state']),
            State(name='unload_rack', on_enter=['request_unload_rack_job','_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_rack', conditions='is_batch_assigned', unless='is_batch_at_station')
        self.machine.add_transition('process_state_transitions',source='load_rack',dest='load_sample', conditions='is_station_job_ready', after=['update_batch_loc_to_station','inc_samples_count'])
        
        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_sample', conditions=['is_batch_assigned','is_batch_at_station'], after='inc_samples_count')

        if self.batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='=', after='inc_samples_count', conditions='is_station_job_ready', unless='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions=['are_all_samples_loaded','is_station_job_ready'])

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample',conditions='is_station_job_ready', before='process_batch', after='dec_samples_count')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='=', conditions='is_station_job_ready', unless='are_all_samples_unloaded', after='dec_samples_count')

            if self._rack_unload_task is None:
                self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions=['are_all_samples_unloaded','is_station_job_ready'])
            else:
                self.machine.add_transition('process_state_transitions', source='unload_sample',dest='unload_rack', conditions=['are_all_samples_unloaded','is_station_job_ready'])
                self.machine.add_transition('process_state_transitions', source='unload_rack',dest='final_state', conditions='is_station_job_ready', before='update_batch_loc_to_robot')

        else:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='is_station_job_ready')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_job_ready', before='process_sample')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded', after='inc_samples_count')

            if self._rack_unload_task is None:
                self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions=['are_all_samples_loaded','is_station_job_ready'] , before='reset_station')
            else:
                self.machine.add_transition('process_state_transitions', source='unload_sample',dest='unload_rack', conditions=['are_all_samples_loaded','is_station_job_ready'])
                self.machine.add_transition('process_state_transitions', source='unload_rack',dest='final_state', conditions='is_station_job_ready' , before=['update_batch_loc_to_robot','reset_station'])

    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def are_all_samples_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def are_all_samples_unloaded(self):
        return self._station.loaded_samples == 0

    def is_batch_assigned(self):
        return len(self._station.assigned_batches) > 1

    def is_batch_at_station(self):
        return self._station.assigned_batches[0].location == self._station.location

    def update_batch_loc_to_station(self):
        self._station.assigned_batches[0].location = self._station.location

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batches[0].location = Location(-1,-1,f'{last_executed_robot_op.robot_stamp}/Deck')

    def inc_samples_count(self):
        self._station.load_sample()

    def dec_samples_count(self):
        self._station.unload_sample()

    def reset_station(self):
        while(self._station.loaded_samples > 0):
          self._station.unload_sample()

    def request_load_vial_job(self):
        sample_index = self._station.loaded_samples + 1 # because on_enter is before 'after' thus this we add 1 to start from 1 instad of zero
        self._station.set_robot_job(RobotTaskOpDescriptor.from_args(name=self._vial_load_task, params=[sample_index]))

    def request_load_rack_job(self):
        self._station.set_robot_job(KukaLBRTask(self._rack_load_task,[False,self._batch_index],RobotTaskType.UNLOAD_FROM_ROBOT,self._station.location))

    def request_unload_rack_job(self):
        self._station.set_robot_job(KukaLBRTask(self._rack_unload_task,[False,self._batch_index],RobotTaskType.LOAD_TO_ROBOT, self._station.location))

    def request_operation(self):
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = StationFactory.create_op_from_dict(current_op_dict)
        self._station.set_station_op(current_op)

    def request_unload_vial_job(self):
        sample_index = self._station.loaded_samples
        self._station.set_robot_job(RobotTaskOpDescriptor.from_args(name=self._vial_load_task, params=[sample_index]))

    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batches[0].add_station_op_to_current_sample(last_operation_op)
                self._station.assigned_batches[0].process_current_sample()
        self.operation_complete = True

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[0].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[0].process_current_sample()
    
    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



