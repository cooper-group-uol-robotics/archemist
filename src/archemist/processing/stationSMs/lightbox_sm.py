from multiprocessing import Condition
from unittest import TestCase
from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.stations.light_box_station import VialProcessingOpDescriptor,ColourDescriptor
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.util import Location
import archemist.persistence.objectConstructor

class LightBoxSM():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self.rack_index = 1
        self.operation_complete = False
        self.batch_mode = args['batch_mode']
        #self.rack_load_task = args['rack_load_task']
        #self.rack_unload_task = args['rack_unload_task']
        #self.vial_load_task = args['vial_load_task']
        #self.vial_unload_task = args['vial_unload_task']


        ''' States '''
        states = [State(name='init_state', on_enter='_print_state'), 
            State(name='load_sample', on_enter=['request_load_vial_job','_print_state']),
            State(name='unload_sample', on_enter='request_unload_vial_job'),
            State(name='station_process', on_enter=['request_process_data_job', '_print_state']),
            State(name='final_state', on_enter='finalize_batch_processing')]
            
            
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_sample', conditions='is_batch_assigned', after='inc_samples_count')

        # load_sample transitions
        self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='is_station_job_ready')

        # station_process transitions
        self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_job_ready', before='process_sample')

        # unload_sample transitions
        self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded', after='inc_samples_count')

        self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions=['are_all_samples_loaded','is_station_job_ready'] , before='reset_station')


    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def are_all_samples_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def is_batch_assigned(self):
        return self._station.assigned_batch is not None

    def reset_station(self):
        while(self._station.loaded_samples > 0):
          self._station.unload_sample()
   
    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        self.operation_complete = True

    def request_load_vial_job(self):
        sample_index = self._station.loaded_samples + 1 # because on_enter is before 'after' thus this we add 1 to start from 1 instad of zero
        self._station.set_robot_job(KukaLBRTask('loadVial',[False,sample_index], self._station.location, RobotOutputDescriptor()))

    def request_unload_vial_job(self):
        sample_index = self._station.loaded_samples
        self._station.set_robot_job(KukaLBRTask('unloadVial',[False,sample_index], self._station.location, RobotOutputDescriptor()))

    def inc_samples_count(self):
        self._station.load_sample()

    def finalize_batch_processing(self):
        self._station.process_assigned_batch()
        self.to_init_state()
        
    def request_process_data_job(self):
        self._station.set_station_op(VialProcessingOpDescriptor(dict(), ColourDescriptor()))

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batch.add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batch.process_current_sample()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')


