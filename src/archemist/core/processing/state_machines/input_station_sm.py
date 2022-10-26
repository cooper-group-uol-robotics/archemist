from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.core.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.core.processing.state_machines.base_sm import BaseSm

class InputStationSm(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='pickup_batch', on_enter=['request_pickup_batch', '_print_state']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='pickup_batch', prepare='update_assigned_batches', conditions='all_batches_assigned')
        self.machine.add_transition('process_state_transitions',source='pickup_batch',dest='removed_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions',source='removed_batch_update',dest='pickup_batch', unless='are_all_batches_unloaded', conditions='is_station_job_ready')

        # finalise picking up batch
        self.machine.add_transition('process_state_transitions', source='removed_batch_update',dest='final_state', conditions=['is_station_job_ready','are_all_batches_unloaded'])

    def update_assigned_batches(self):
        self._current_batches_count = len(self._station.assigned_batches)

    def request_pickup_batch(self):
        robot_job = KukaLBRTask.from_args(name='PickupInputRack',params=[False,self._current_batch_index+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



