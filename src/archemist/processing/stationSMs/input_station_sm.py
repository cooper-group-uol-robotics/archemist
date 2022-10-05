from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.state.stations.input_station import InputStationPickupOp, InputStationResultDescriptor
from archemist.util import Location
import archemist.persistence.objectConstructor

class InputStationSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self._loaded_racks = 0
        self._current_rack_index = -1
        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='pickup_rack', on_enter=['request_pickup_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='pickup_rack', prepare='update_batch_addition', conditions='all_batches_assigned')
        self.machine.add_transition('process_state_transitions',source='pickup_rack',dest='=', unless='are_all_racks_unloaded', conditions='is_station_job_ready', before='update_batch_loc_to_robot')

        # finalise picking up rack
        self.machine.add_transition('process_state_transitions', source='pickup_rack',dest='final_state', conditions=['is_station_job_ready','are_all_racks_unloaded'], before='update_batch_loc_to_robot')



    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def all_batches_assigned(self):
        return not self._station.has_free_batch_capacity()

    def update_batch_addition(self):
        self._loaded_racks = len(self._station.assigned_batches)

    def are_all_racks_unloaded(self):
        return self._loaded_racks == 0

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batches[self._current_rack_index].location = Location(-1,-1,f'{last_executed_robot_op.output.executing_robot}/Deck')

    def request_pickup_rack(self):
        self._loaded_racks -= 1
        self._current_rack_index += 1
        robot_job = KukaLBRTask('PickupInputRack',[False,self._current_rack_index+1],RobotTaskType.LOAD_TO_ROBOT, self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_rack_index].id
        self._station.set_robot_job(robot_job, current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._current_rack_index = -1
        self._loaded_racks = 0
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



