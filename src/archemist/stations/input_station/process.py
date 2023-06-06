from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.state.station_process import StationProcess, StationProcessData

    
class InputStationProcess(StationProcess):

    def __init__(self, station: Station, process_data: StationProcessData, **kwargs): 
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'pickup_batch'},
            {'source':'pickup_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'pickup_batch', 'unless':'are_all_batches_unloaded'},
            {'source':'removed_batch_update','dest':'final_state', 'conditions':'are_all_batches_unloaded'},
        ]

        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def update_unloaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)
        self._process_data.status['batch_index'] += 1

    def request_pickup_batch(self):
        batch_offset = self._process_data.processing_slot*self._station.process_batch_capacity
        batch_index = self._process_data.status['batch_index']
        perform_6p = False
        robot_op = KukaLBRTask.from_args(name='PickupInputRack',params=[perform_6p,batch_offset + batch_index+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()

    ''' transitions callbacks'''

    def are_all_batches_unloaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)
        
class CrystalWorkflowInputStationProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='pickup_batch', on_enter='request_pickup_batch'),
            State(name='pickup_pxrd_plate', on_enter='request_pickup_pxrd_plate'),
            State(name='removed_batch_update', on_enter='update_unloaded_batch'),
            State(name='final_state', on_enter='finalize_batch_processing')]
        
        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'pickup_batch'},
            {'source':'pickup_batch','dest':'pickup_pxrd_plate', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pickup_pxrd_plate','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'pickup_batch', 'unless':'are_all_batches_unloaded'},
            { 'source':'removed_batch_update','dest':'final_state', 'conditions':'are_all_batches_unloaded'},
        ]

        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def request_pickup_batch(self):
        batch_index = self._process_data.status['batch_index']
        perform_6p = False
        robot_op = KukaLBRTask.from_args(name='PickupEightWRack',params=[perform_6p,batch_index+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def request_pickup_pxrd_plate(self):
        batch_index = self._process_data.status['batch_index']
        perform_6p = False
        robot_op = KukaLBRTask.from_args(name='PickupPXRDRack',params=[perform_6p],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def update_unloaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)
        self._process_data.status['batch_index'] += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()

    ''' transitions callbacks '''
    def are_all_batches_unloaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)
