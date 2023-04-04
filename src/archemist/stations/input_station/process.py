from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.state.station_process import StationProcess, StationProcessData

class InputStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)        

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'prepare':'update_assigned_batches', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'pickup_batch', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'pickup_batch','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'removed_batch_update','dest':'pickup_batch', 'unless':'are_all_batches_unloaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','are_all_batches_unloaded']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def update_assigned_batches(self):
        self._status['batches_count'] = len(self._station.assigned_batches)

    def request_pickup_batch(self):
        robot_job = KukaLBRTask.from_args(name='PickupInputRack',params=[False,self._status['batch_index']+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self.to_init_state()
        
class CrystalWorkflowInputStationSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)        

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='pickup_pxrd_plate', on_enter=['request_pickup_pxrd_plate']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function,'source':'init_state','dest':'disable_auto_functions', 'prepare':'update_assigned_batches', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'pickup_batch', 'prepare':'update_assigned_batches', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'pickup_batch','dest':'pickup_pxrd_plate', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'pickup_pxrd_plate','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'removed_batch_update','dest':'pickup_batch', 'unless':'are_all_batches_unloaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'enable_auto_functions', 'conditions':['is_station_job_ready','are_all_batches_unloaded']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':'is_station_job_ready'}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def update_assigned_batches(self):
        self._status['batches_count'] = len(self._station.assigned_batches)

    def request_pickup_batch(self):
        robot_job = KukaLBRTask.from_args(name='PickupEightWRack',params=[False,self._status['batch_index']+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_pickup_pxrd_plate(self):
        robot_job = KukaLBRTask.from_args(name='PickupPXRDRack',params=[False],
                                        type=RobotTaskType.MANIPULATION, location=self._station.location)
        self._station.request_robot_op(robot_job)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.to_init_state()

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

class RefactoredInputStationSm(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs): 
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]
        
        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'disable_auto_functions'},
            {'source':'disable_auto_functions','dest':'pickup_batch', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pickup_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'pickup_batch', 'unless':'are_all_batches_unloaded'},
            {'source':'removed_batch_update','dest':'enable_auto_functions', 'conditions':'are_all_batches_unloaded'},
            {'source':'enable_auto_functions','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]

        super().__init__(station, process_data, states, transitions)

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0

    def update_unloaded_batch(self):
        self._process_data.status['batch_index'] += 1

    def are_all_batches_unloaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)

    def request_pickup_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='PickupInputRack',params=[False,batch_index+1],
                                        type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op, current_batch_id)

    def request_disable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def request_enable_auto_functions(self):
        robot_op = KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False])
        self.request_robot_op(robot_op)

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()