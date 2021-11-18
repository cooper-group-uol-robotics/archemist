from transitions import Machine, State
from state.station import Station, StationState
from state.robot import TransportBatchOpDescriptor, RackMoveOpDescriptor
from util import Location

class RackOnStationSm():
    
    def __init__(self, batch_mode: bool, station:Station):
        self.batch_mode = batch_mode
        self.station = station
        self.current_job = None

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='retrieve_batch', on_enter='request_retrieve_job', on_exit='reset_current_job'), 
            State(name='place_batch', on_enter='request_retrieve_job', on_exit='reset_current_job'),
            State(name='load_sample', on_enter='request_retrieve_job', on_exit='reset_current_job'), 
            State(name='station_process', on_enter='request_retrieve_job', on_exit='reset_current_job'),
            State(name='unload_sample', on_enter='request_retrieve_job', on_exit='reset_current_job'),
            State(name='final_state')]
        self.machine = Machine(self, states=states, initial='start')

        self.machine.on_enter_retrieve_batch('request_retrieve_job')
        self.machine.on_exit_retrieve_batch('reset_current_job')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='retreive_batch',unless='is_batch_at_station')
        self.machine.add_transition('process_state_transitions',source='init_state',dest='place_batch', conditions='is_batch_at_station')
        
        # retrieve_batch transitions
        self.machine.add_transition('process_state_transitions',source='retrieve_batch', dest='place_batch')

        # place_batch transitions
        self.machine.add_transition('process_state_transitions',source='place_batch', dest='load_sample')

        if batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='=', unless='are_all_samples_loaded', after='inc_samples_count')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_process_complete')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions='are_all_samples_unloaded')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='=', unless='are_all_samples_unloaded', after='dec_samples_count')
        else:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', after='inc_samples_count')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_process_complete')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions='are_all_samples_loaded', after='reset_station')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', unless='are_all_samples_loaded')

        
    

    def are_all_samples_loaded(self):
        return self.station.loaded_samples == self.station.assigned_batch.num_samples

    def are_all_samples_unloaded(self):
        return self.station.loaded_samples == 0

    def is_station_process_complete(self):
        return self.station.state == StationState.PROCESSING_COMPLETE

    def inc_samples_count(self):
        self.station.load_sample()

    def dec_samples_count(self):
        self.station.unload_sample()

    def reset_station(self):
        while(self.station.loaded_samples > 0):
          self.station.unload_sample()

    def request_retrieve_job(self):
        self.current_job = TransportBatchOpDescriptor('', self.station.assigned_batch.location, None)

    def reset_current_job(self):
        self.current_job = None

    def request_place_rack_job(self):
        self.current_job = RackMoveOpDescriptor('',self.station.assigned_batch.location, Location())



