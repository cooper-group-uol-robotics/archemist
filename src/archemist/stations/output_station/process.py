from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.state.station_process import StationProcess, StationProcessData

class OutputStationProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='place_batch', on_enter='request_place_batch'),
            State(name='added_batch_update', on_enter='update_loaded_batch'),
            State(name='final_state', on_enter='finalize_batch_processing')]

        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'place_batch'},
            {'source':'place_batch','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update','dest':'place_batch', 'unless':'are_all_batches_loaded'},
            { 'source':'added_batch_update','dest':'final_state', 'conditions':'are_all_batches_loaded'}
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''
    
    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def request_place_batch(self):
        batch_offset = self._process_data.processing_slot*self._station.process_batch_capacity
        batch_index = self._process_data.status['batch_index']
        perform_6p = False # this will be later evaluated by the KMRiiwa handler
        robot_job = KukaLBRTask.from_args(name='PlaceRack',
                                          params=[perform_6p,batch_offset+batch_index+1, batch_index+1],
                                        type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_job,current_batch_id)

    def update_loaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)
        self._process_data.status['batch_index'] += 1

    def finalize_batch_processing(self):
        for batch in self._process_data.batches:
            self._station.process_assinged_batch(batch)

    ''' transitions callbacks'''

    def are_all_batches_loaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)

    



