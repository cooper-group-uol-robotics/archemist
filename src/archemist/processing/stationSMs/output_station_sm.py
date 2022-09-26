from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.state.stations.input_station import InputStationPickupOp, InputStationResultDescriptor
from archemist.util import Location
import archemist.persistence.objectConstructor

class OutputStationSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='place_rack', on_enter=['request_place_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='place_rack', conditions='is_batch_assigned')

        # finalise picking up rack
        self.machine.add_transition('process_state_transitions', source='place_rack',dest='final_state', conditions='is_station_job_ready', before='update_batch_loc_to_robot')



    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def is_batch_assigned(self):
        return self._station.assigned_batch is not None

    def update_batch_loc_to_robot(self):
        self._station.assigned_batch.location = self._station.location

    def request_place_rack(self):
        self._station.set_robot_job(KukaLBRTask('PlaceRack',[False,1],RobotTaskType.UNLOAD_FROM_ROBOT, self._station.location))

    def finalize_batch_processing(self):
        self._station.process_assigned_batch()
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



