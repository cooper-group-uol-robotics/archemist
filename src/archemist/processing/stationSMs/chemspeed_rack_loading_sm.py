from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.state.stations.chemspeed_flex_station import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSStartJobOpDescriptor, CSJobOutputDescriptor
from archemist.util import Location

class ChemSpeedRackLoadingSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self.batch_mode = args['batch_mode']
        self._load_frame = args['load_frame']
        self._racks_no = args['no_racks']
        self._current_rack = 0


        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door', '_print_state']), 
            State(name='chemspeed_process', on_enter=['start_operation', '_print_state']),
            State(name='load_rack', on_enter=['request_load_rack', '_print_state']),
            State(name='close_chemspeed_door', on_enter=['request_close_door', '_print_state']), 
            State(name='unload_rack', on_enter=['request_unload_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='open_chemspeed_door', conditions='is_batch_assigned')

        # load rack into the chemspeed
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='load_rack', conditions='is_station_job_ready')

        # close door after loading rack
        self.machine.add_transition('process_state_transitions', source='load_rack',dest='close_chemspeed_door', conditions='is_station_job_ready')

        # process batch after closing door
        self.machine.add_transition('process_state_transitions', source='close_chemspeed_door',dest='chemspeed_process', conditions='is_station_job_ready')

        # open door after processing
        self.machine.add_transition('process_state_transitions', source='chemspeed_process',dest='open_chemspeed_door', conditions='is_station_job_ready', after='set_station_to_processing')

        # unload after openning door
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='unload_rack', conditions='is_station_job_ready')

        # complete
        self.machine.add_transition('process_state_transitions', source='unload_rack',dest='final_state', conditions=['are_all_samples_loaded','is_station_job_ready'])



    def is_station_job_ready(self):
        return self._station.state == StationState.PROCESSING and not self._station.has_robot_job()

    def is_station_operation_complete(self):
        return self._station.state == StationState.OPERATION_COMPLETE

    def is_batch_assigned(self):
        return self._station.assigned_batch is not None

    def set_station_to_processing(self):
        self._station.set_to_processing()

    def request_open_door(self):
        self._station.set_station_op(CSOpenDoorOpDescriptor())

    def request_close_door(self):
        self._station.set_station_op(CSCloseDoorOpDescriptor())

    def request_load_rack(self):
        self._station.set_robot_job(KukaLBRTask('LoadChemSpeed',['false','1'], self._station.location, RobotOutputDescriptor()))

    def request_unload_rack(self):
        self._station.set_robot_job(KukaLBRTask('UnloadChemSpeed',['false','1'], self._station.location, RobotOutputDescriptor()))

    def start_operation(self):
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        #current_op = ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        self._station.set_station_op(CSStartJobOpDescriptor())

    def finalize_batch_processing(self):
        self._station.process_assigned_batch()
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



