from typing import Dict
from transitions import Machine, State
from archemist.state.station import Station
from archemist.state.robot import RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.processing.state_machines.base_sm import BaseSm

class OutputStationSm(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='place_rack', on_enter=['request_place_rack', '_print_state']),
            State(name='added_batch_update', on_enter=['update_loaded_batch', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='place_rack', conditions='all_batches_assigned')
        self.machine.add_transition('process_state_transitions',source='place_rack',dest='added_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions',source='added_batch_update',dest='place_rack', unless='are_all_batches_loaded', conditions='is_station_job_ready')

        # finalise picking up rack
        self.machine.add_transition('process_state_transitions', source='added_batch_update',dest='final_state', conditions=['is_station_job_ready','are_all_batches_loaded'])

    def request_place_rack(self):
        robot_job = KukaLBRTask.from_args(name='PlaceRack',params=[False,self._current_batch_index+1],
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.set_robot_job(robot_job,current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



