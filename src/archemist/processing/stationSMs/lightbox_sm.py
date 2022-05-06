from multiprocessing import Condition
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
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='station_off', on_enter=['waiting_station_on', '_print_state']), 
            State(name='stand_by', on_enter=['request_stand_by', '_print_state']),
            State(name='take_image', on_enter=['request_take_image', '_print_state'])]
          
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state wake up station transition
        self.machine.add_transition('process_state_transitions',source='station_off',dest='stand_by', conditions='is_batch_assigned')

        # turn on camera for imaging
        self.machine.add_transition('process_state_transitions', source='stand_by',dest='take_image', before='update_batch_loc_to_station' ,conditions='is_station_job_ready')

        # Prepare for the next vial 
        self.machine.add_transition('process_state_transitions', source='take_image',dest='stand_by', conditions='is_station_operation_complete', before='update_batch_loc_to_robot')

        # back to init_state sleep state 
        self.machine.add_transition('process_state_transitions', source='stand_by',dest='station_off', conditions='is_station_operation_complete')


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

    


