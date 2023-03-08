from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from .state import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSCSVJobOpDescriptor, CSProcessingOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location

class ChemSpeedRackSm(StationProcessFSM):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'operation_complete' not in self._status.keys():
            self._status['operation_complete'] = False

        ''' States '''
        states = [ State(name='init_state'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions']),
            State(name='navigate_to_chemspeed', on_enter=['request_navigate_to_chemspeed']),
            State(name='retreat_from_chemspeed', on_enter=['request_navigate_to_chemspeed']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions']), 
            State(name='chemspeed_process', on_enter=['request_process_operation']),
            State(name='load_batch', on_enter=['request_load_batch']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='close_chemspeed_door', on_enter=['request_close_door']), 
            State(name='unload_batch', on_enter=['request_unload_batch']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'trigger':self._trigger_function, 'source':'init_state', 'dest': 'disable_auto_functions', 'conditions':'all_batches_assigned'},
            {'trigger':self._trigger_function,'source':'disable_auto_functions','dest':'navigate_to_chemspeed', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function,'source':'navigate_to_chemspeed','dest':'open_chemspeed_door', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'open_chemspeed_door','dest':'load_batch', 'unless':'is_station_operation_complete' ,'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'load_batch','dest':'added_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'load_batch', 'unless':'are_all_batches_loaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'added_batch_update','dest':'close_chemspeed_door', 'conditions':['is_station_job_ready','are_all_batches_loaded']},
            {'trigger':self._trigger_function,'source':'close_chemspeed_door','dest':'enable_auto_functions', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'retreat_from_chemspeed', 'unless':'is_station_operation_complete', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'retreat_from_chemspeed','dest':'chemspeed_process', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'chemspeed_process','dest':'disable_auto_functions', 'conditions':'is_station_job_ready', 'before':'process_batches'},
            {'trigger':self._trigger_function, 'source':'open_chemspeed_door','dest':'unload_batch', 'conditions':['is_station_operation_complete','is_station_job_ready']},
            {'trigger':self._trigger_function, 'source':'unload_batch','dest':'removed_batch_update', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'unload_batch', 'unless':'are_all_batches_unloaded', 'conditions':'is_station_job_ready'},
            {'trigger':self._trigger_function, 'source':'removed_batch_update','dest':'close_chemspeed_door', 'conditions':['is_station_job_ready','are_all_batches_unloaded']},
            {'trigger':self._trigger_function, 'source':'enable_auto_functions','dest':'final_state', 'conditions':['is_station_job_ready','is_station_operation_complete']}
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def is_station_operation_complete(self):
        return self._status['operation_complete']

    def request_open_door(self):
        self._station.assign_station_op(CSOpenDoorOpDescriptor.from_args())

    def request_close_door(self):
        self._station.assign_station_op(CSCloseDoorOpDescriptor.from_args())

    def request_load_batch(self):
        robot_job = KukaLBRTask.from_args(name='LoadChemSpeed',params=[True,self._status['batch_index']+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_batch(self):
        robot_job = KukaLBRTask.from_args(name='UnloadChemSpeed',params=[False,self._status['batch_index']+1],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        self._station.request_robot_op(robot_job,current_batch_id)
        

    def request_navigate_to_chemspeed(self):
        self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def request_process_operation(self):
        current_op = self._station.assigned_batches[-1].recipe.get_current_task_op()
        if isinstance (current_op,CSCSVJobOpDescriptor):
            contacnated_csv = ''
            for batch in self._station.assigned_batches:
                current_op = batch.recipe.get_current_task_op()
                contacnated_csv += current_op.csv_string
            current_op.csv_string = contacnated_csv
            self._station.assign_station_op(current_op)
        elif isinstance(current_op, CSProcessingOpDescriptor):
            self._station.assign_station_op(current_op)


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



