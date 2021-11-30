from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor
from archemist.util import Location

class StationLoadingSm():
    
    def __init__(self, args: dict):
        self.batch_mode = args['batch_mode']
        self._load_frame = args['load_frame']

        self._station = None

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='load_sample', on_enter='request_load_vial_job'), 
            State(name='station_process', on_enter='start_operation'),
            State(name='unload_sample', on_enter='request_unload_vial_job'),
            State(name='final_state', on_enter='finalize_batch_processing')]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='load_sample', conditions='is_batch_assigned', after='inc_samples_count')

        if self.batch_mode:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='=', after='inc_samples_count', conditions='is_station_job_ready', unless='are_all_samples_loaded')
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions=['are_all_samples_loaded','is_station_job_ready'])

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_operation_complete', after=['dec_samples_count','set_station_to_processing'])

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='=', conditions='is_station_job_ready', unless='are_all_samples_unloaded', after='dec_samples_count')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions=['are_all_samples_unloaded','is_station_job_ready'])
        else:
            # load_sample transitions
            self.machine.add_transition('process_state_transitions', source='load_sample',dest='station_process', conditions='is_station_job_ready')

            # station_process transitions
            self.machine.add_transition('process_state_transitions', source='station_process',dest='unload_sample', conditions='is_station_operation_complete', after='set_station_to_processing')

            # unload_sample transitions
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='load_sample', conditions='is_station_job_ready', unless='are_all_samples_loaded', after='inc_samples_count')
            self.machine.add_transition('process_state_transitions', source='unload_sample',dest='final_state', conditions=['are_all_samples_loaded','is_station_job_ready'] , before='reset_station')
        
    def set_station(self, station:Station):
        self._station = station

    def is_station_job_ready(self):
        return self._station.state == StationState.PROCESSING and not self._station.has_robot_job()

    def are_all_samples_loaded(self):
        return self._station.loaded_samples == self._station.assigned_batch.num_samples

    def are_all_samples_unloaded(self):
        return self._station.loaded_samples == 0

    def is_station_operation_complete(self):
        return self._station.state == StationState.OPERATION_COMPLETE

    def is_batch_assigned(self):
        return self._station.state == StationState.PROCESSING

    def inc_samples_count(self):
        self._station.load_sample()

    def dec_samples_count(self):
        self._station.unload_sample()

    def set_station_to_processing(self):
        self._station.state = StationState.PROCESSING

    def reset_station(self):
        while(self._station.loaded_samples > 0):
          self._station.unload_sample()

    def request_load_vial_job(self):
        sample_index = self._station.loaded_samples + 1 # because on_enter is before 'after' thus this we add 1 to start from 1 instad of zero
        sample_target_location = self._station.create_location_from_frame(self._load_frame)
        self._station.set_robot_job(MoveSampleOp(sample_index, self._station.assigned_batch.location, sample_target_location, RobotOutputDescriptor()))

    def start_operation(self):
        self._station.state = StationState.WAITING_ON_OPERATION

    def request_unload_vial_job(self):
        sample_index = self._station.loaded_samples
        sample_start_location = self._station.create_location_from_frame(self._load_frame)
        self._station.set_robot_job(MoveSampleOp(sample_index, sample_start_location, self._station.assigned_batch.location, RobotOutputDescriptor()))

    def finalize_batch_processing(self):
        self._station.process_assigned_batch()
        self.to_init_state()



