from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.util import Location
import archemist.persistence.objectConstructor

class OutputStationSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self._loaded_racks = 0
        self._current_rack_index = -1
        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='place_rack', on_enter=['request_place_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='place_rack', conditions='all_batches_assigned')
        self.machine.add_transition('process_state_transitions',source='place_rack',dest='=', unless='are_all_racks_loaded', conditions='is_station_job_ready',before='update_batch_loc_to_station')

        # finalise picking up rack
        self.machine.add_transition('process_state_transitions', source='place_rack',dest='final_state', conditions=['is_station_job_ready','are_all_racks_loaded'], before='update_batch_loc_to_station')


    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def all_batches_assigned(self):
        return not self._station.has_free_batch_capacity()

    def update_batch_loc_to_station(self):
        self._station.assigned_batches[self._current_rack_index].location = self._station.location

    def are_all_racks_loaded(self):
        return self._loaded_racks == self._station.batch_capacity

    def request_place_rack(self):
        self._loaded_racks += 1
        self._current_rack_index += 1
        robot_job = KukaLBRTask('PlaceRack',[False,self._current_rack_index+1],RobotTaskType.UNLOAD_FROM_ROBOT, self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_rack_index].id
        self._station.set_robot_job(robot_job,current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._current_rack_index = -1
        self._loaded_racks = 0
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



