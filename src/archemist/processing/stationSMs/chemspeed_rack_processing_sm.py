from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.state.stations.chemspeed_flex_station import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSProcessingOpDescriptor, CSJobOutputDescriptor
from archemist.util import Location
import archemist.persistence.objectConstructor

class ChemSpeedRackProcessingSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self.batch_mode = args['batch_mode']
        self.operation_complete = False

        #self._load_frame = args['load_frame']
        #self._racks_no = args['no_racks']
        #self._current_rack = 0


        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door', '_print_state']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions', '_print_state']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions', '_print_state']), 
            State(name='chemspeed_process', on_enter=['request_process_operation', '_print_state']),
            State(name='load_rack', on_enter=['request_load_rack', '_print_state']),
            State(name='close_chemspeed_door', on_enter=['request_close_door', '_print_state']), 
            State(name='unload_rack', on_enter=['request_unload_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='open_chemspeed_door', conditions='is_batch_assigned')

        # load rack into the chemspeed
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='load_rack', unless='is_station_operation_complete' ,conditions='is_station_job_ready')

        # close door after loading rack
        self.machine.add_transition('process_state_transitions', source='load_rack',dest='close_chemspeed_door', conditions='is_station_job_ready', before='update_batch_loc_to_station')

        # process batch after closing door
        self.machine.add_transition('process_state_transitions', source='close_chemspeed_door',dest='chemspeed_process', unless='is_station_operation_complete', conditions='is_station_job_ready')

        # open door after processing
        self.machine.add_transition('process_state_transitions', source='chemspeed_process',dest='open_chemspeed_door', conditions='is_station_job_ready', before='process_batch')

        # unload after openning door
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='unload_rack', conditions=['is_station_operation_complete','is_station_job_ready'])

        # close door after unloading
        self.machine.add_transition('process_state_transitions', source='unload_rack',dest='close_chemspeed_door', conditions='is_station_job_ready', before='update_batch_loc_to_robot')

        # complete
        self.machine.add_transition('process_state_transitions', source='close_chemspeed_door',dest='final_state', conditions=['is_station_job_ready','is_station_operation_complete'])



    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def is_station_operation_complete(self):
        return self.operation_complete

    def is_batch_assigned(self):
        return self._station.assigned_batch is not None

    def update_batch_loc_to_station(self):
        self._station.assigned_batch.location = self._station.location

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batch.location = Location(-1,-1,f'{last_executed_robot_op.output.executing_robot}/Deck')

    def request_open_door(self):
        self._station.set_station_op(CSOpenDoorOpDescriptor(dict(), CSJobOutputDescriptor()))

    def request_close_door(self):
        self._station.set_station_op(CSCloseDoorOpDescriptor(dict(), CSJobOutputDescriptor()))

    def request_load_rack(self):
        self._station.set_robot_job(KukaLBRTask('LoadChemSpeed',[False,1], self._station.location, RobotOutputDescriptor())) #TODO rack index can be passed

    def request_unload_rack(self):
        self._station.set_robot_job(KukaLBRTask('UnloadChemSpeed',[False,1], self._station.location, RobotOutputDescriptor()))

    def request_disable_auto_functions(self):
        self._station.set_robot_job(KukaLBRTask('DiableAutoFunctions',[False,1], self._station.location, RobotOutputDescriptor()))

    def request_enable_auto_functions(self):
        self._station.set_robot_job(KukaLBRTask('EnableAutoFunctions',[False,1], self._station.location, RobotOutputDescriptor()))

    def request_process_operation(self):
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = archemist.persistence.objectConstructor.ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        self._station.set_station_op(current_op)

    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        #TODO if we have multiple racks loop through them
        for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batch.add_station_op_to_current_sample(last_operation_op)
                self._station.assigned_batch.process_current_sample()
        self.operation_complete = True

    def finalize_batch_processing(self):
        self._station.process_assigned_batch()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



