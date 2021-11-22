from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import TransportBatchOpDescriptor, RackMoveOpDescriptor, VialMoveOpDescriptor, RobotOutputDescriptor
from archemist.util import Location

class RackOnStationSm():
    
    def __init__(self, args: dict):
        self.batch_mode = args['batch_mode']
        self._place_frame = args['place_frame']
        self._load_frame = args['load_frame']
        self._unload_frame = args['unload_frame']

        self._station = None

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='retrieve_batch', on_enter='request_retrieve_job', on_exit='reset_current_job'), 
            State(name='place_batch', on_enter='request_place_rack_job', on_exit='reset_current_job'),
            State(name='load_sample', on_enter='request_load_job', on_exit='reset_current_job'), 
            State(name='station_process', on_enter='start_processing'),
            State(name='unload_sample', on_enter='request_unload_job', on_exit='reset_current_job'),
            State(name='final_state', on_enter='finish_batch')]
        
        self.machine = Machine(self, states=states, initial='start')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='retreive_batch',unless='is_batch_at_station')
        self.machine.add_transition('process_state_transitions',source='init_state',dest='place_batch', conditions='is_batch_at_station')
        
        # retrieve_batch transitions
        self.machine.add_transition('process_state_transitions',source='retrieve_batch', dest='place_batch')

        # place_batch transitions
        self.machine.add_transition('process_state_transitions',source='place_batch', dest='load_sample')

        if self.batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='are_all_samples_loaded', prepare='set_station')
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='=', unless='are_all_samples_loaded', after='inc_samples_count', prepare='set_station')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_process_complete', prepare='set_station')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions='are_all_samples_unloaded', prepare='set_station')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='=', unless='are_all_samples_unloaded', prepare='set_station', after='dec_samples_count')
        else:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', prepare='set_station', after='inc_samples_count')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_process_complete', prepare='set_station')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions='are_all_samples_loaded', after='reset_station', prepare='set_station')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', unless='are_all_samples_loaded', prepare='set_station')

        
    def set_station(self, station:Station):
        self._station = station

    def are_all_samples_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def are_all_samples_unloaded(self):
        return self._station.loaded_samples == 0

    def is_station_process_complete(self):
        return self._station.state == StationState.PROCESSING_COMPLETE

    def is_batch_at_station(self):
        return self._station.assigned_batch.location.get_map_coordinates() == self._station.location.get_map_coordinates() 

    def inc_samples_count(self):
        self._station.load_sample()

    def dec_samples_count(self):
        self._station.unload_sample()

    def reset_station(self):
        while(self._station.loaded_samples > 0):
          self._station.unload_sample()

    def request_retrieve_job(self):
        self._station.set_robot_job(TransportBatchOpDescriptor('', self._station.assigned_batch.location, RobotOutputDescriptor()))

    def request_place_rack_job(self):
        self._station.set_robot_job(RackMoveOpDescriptor('',self._station.assigned_batch.location, Location(self._station.location.node_id, self._station.location.graph_id, self._place_frame), RobotOutputDescriptor()))

    def request_load_vial_job(self):
        self._station.set_robot_job(VialMoveOpDescriptor('',self._station.assigned_batch.location,Location(self._station.location.node_id, self._station.location.graph_id, self._load_frame), RobotOutputDescriptor()))

    def start_processing(self):
        self._station.state = StationState.PROCESSING

    def request_unload_vial_job(self):
        self._station.set_robot_job(VialMoveOpDescriptor('',self._station.assigned_batch.location, Location(self._station.location.node_id, self._station.location.graph_id, self._unload_frame), RobotOutputDescriptor()))

    def finish_batch(self):
        self._station.batch_completed()



