from typing import Dict
from transitions import State
from .state import PXRDStatus
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.util import Location

class PXRDSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='open_pxrd_door', on_enter=['request_open_pxrd_door']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']), 
            State(name='pxrd_process', on_enter=['request_pxrd_process']),
            State(name='load_pxrd', on_enter=['request_load_pxrd']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='close_pxrd_door', on_enter=['request_close_pxrd_door']), 
            State(name='unload_pxrd', on_enter=['request_unload_pxrd']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'open_pxrd_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'open_pxrd_door','dest':'load_pxrd', 'unless':'is_station_operation_complete' ,
                'conditions':'is_station_job_ready', 'before':'set_doors_to_open'},
            {'trigger':self._trigger_function, 'source':'load_pxrd','dest':'added_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'close_pxrd_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'close_pxrd_door','dest':'enable_auto_functions', 'conditions':'is_station_job_ready',
                'before':'set_doors_to_closed'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'pxrd_process', 'unless':'is_station_operation_complete', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'pxrd_process','dest':'disable_auto_functions', 'conditions':'is_station_job_ready', 'before':'process_batches'},
            {'trigger':self._trigger_function, 'source':'open_pxrd_door','dest':'unload_pxrd', 'conditions':['is_station_operation_complete','is_station_job_ready']
                ,'before':'set_doors_to_open'},
            {'trigger':self._trigger_function, 'source':'unload_pxrd','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'close_pxrd_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready','is_station_operation_complete']}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_open_pxrd_door(self):
        door_loc = Location(node_id=19, graph_id=1)
        robot_job = KukaLBRTask.from_args(name='OpenDoors',params=[True], 
                                            type=RobotTaskType.MANIPULATION, location=door_loc)
        self._station.request_robot_op(robot_job)

    def request_close_pxrd_door(self):
        door_loc = Location(node_id=19, graph_id=1)
        robot_job = KukaLBRTask.from_args(name='CloseDoors',params=[True], 
                                            type=RobotTaskType.MANIPULATION, location=door_loc)
        self._station.request_robot_op(robot_job)

    def request_load_pxrd(self):
        robot_job = KukaLBRTask.from_args(name='LoadPXRD',params=[True], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_unload_pxrd(self):
        robot_job = KukaLBRTask.from_args(name='UnloadPXRD',params=[True], 
                                            type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def request_pxrd_process(self):
        current_op = self._station.assigned_batches[-1].recipe.get_current_task_op()
        self._station.assign_station_op(current_op)

    def set_doors_to_open(self):
        self._station.status = PXRDStatus.DOORS_OPEN

    def set_doors_to_closed(self):
        self._station.status = PXRDStatus.DOORS_CLOSED
       

    def process_batches(self):
        last_operation_op = self._station.station_op_history[-1]
        for batch in self._station.assigned_batches:
            for _ in range(0, batch.num_samples):
                    batch.add_station_op_to_current_sample(last_operation_op)
                    batch.process_current_sample()
        self._status['operation_complete'] = True

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['operation_complete'] = False
        self._status['batch_index'] = 0
        self.to_init_state()



