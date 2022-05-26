from multiprocessing import Condition
from unittest import TestCase
from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.util import Location
import archemist.persistence.objectConstructor

class LightBoxSM():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self.batch_mode = args['batch_mode']
        self.operation_complete = False

        ''' States '''
        states = [State(name='init_state', on_enter='_print_state'), 
            State(name='load_vial', on_enter=['request_load_vial_job', '_print_state']),
            State(name='unload_vial', on_enter=['request_unload_vial_job', '_print_state']),
            State(name='process_sample', on_enter=['request_process_data_job','_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
            
            
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        if self.batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='=', after='inc_samples_count', conditions='is_station_job_ready', unless='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions=['are_all_samples_loaded','is_station_job_ready'])

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample',conditions='is_station_job_ready', before='process_batch', after='dec_samples_count')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='=', conditions='is_station_job_ready', unless='are_all_samples_unloaded', after='dec_samples_count')



    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def is_sample_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def is_station_operation_complete(self):
        return self.operation_complete

    def are_all_samples_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def is_batch_assigned(self):
        return self._station.assigned_batch is not None

    
    def dec_samples_count(self):
        self._station.unload_sample()

    def reset_station(self):
        while(self._station.loaded_samples > 0):
          self._station.unload_sample()
   
    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        self.operation_complete = True

    def request_load_vial_job(self):
        sample_index = self._station.loaded_samples + 1 # because on_enter is before 'after' thus this we add 1 to start from 1 instad of zero
        sample_target_location = self._station.create_location_from_frame(self._load_frame)
        self._station.set_robot_job(MoveSampleOp(sample_index, self._station.assigned_batch.location, sample_target_location, RobotOutputDescriptor()))

    def request_unload_vial_job(self):
        sample_index = self._station.loaded_samples
        sample_start_location = self._station.create_location_from_frame(self._load_frame)
        self._station.set_robot_job(MoveSampleOp(sample_index, sample_start_location, self._station.assigned_batch.location, RobotOutputDescriptor()))

    #Test 


    


