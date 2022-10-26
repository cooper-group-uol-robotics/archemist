from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.core.state.robots.kukaLBRIIWA import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.core.state.stations.chemspeed_flex_station import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSCSVJobOpDescriptor, CSProcessingOpDescriptor
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.state_machines.base_sm import BaseSm
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location

class ChemSpeedRackSm(BaseSm):
    
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        self.operation_complete = False
        self._current_batch_index = 0

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door', '_print_state']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions', '_print_state']),
            State(name='navigate_to_chemspeed', on_enter=['request_navigate_to_chemspeed', '_print_state']),
            State(name='retreat_from_chemspeed', on_enter=['request_navigate_to_chemspeed', '_print_state']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions', '_print_state']), 
            State(name='chemspeed_process', on_enter=['request_process_operation', '_print_state']),
            State(name='load_batch', on_enter=['request_load_batch', '_print_state']),
            State(name='added_batch_update', on_enter=['update_loaded_batch', '_print_state']),
            State(name='close_chemspeed_door', on_enter=['request_close_door', '_print_state']), 
            State(name='unload_batch', on_enter=['request_unload_batch', '_print_state']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='disable_auto_functions', conditions='all_batches_assigned')

        # disable autofunctions and navigate to the station
        self.machine.add_transition('process_state_transitions',source='disable_auto_functions',dest='navigate_to_chemspeed', conditions='is_station_job_ready')

        # open chemspeed door
        self.machine.add_transition('process_state_transitions',source='navigate_to_chemspeed',dest='open_chemspeed_door', conditions='is_station_job_ready')

        # load all batches into the chemspeed
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='load_batch', unless='is_station_operation_complete' ,conditions='is_station_job_ready')
        # self.machine.add_transition('process_state_transitions', source='load_batch',dest='=', unless='are_all_batches_loaded', conditions='is_station_job_ready', before='update_batch_loc_to_station')
        self.machine.add_transition('process_state_transitions', source='load_batch',dest='added_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions', source='added_batch_update',dest='load_batch', unless='are_all_batches_loaded', conditions='is_station_job_ready')


        # close door after loading batch
        # self.machine.add_transition('process_state_transitions', source='load_batch',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_batches_loaded'], before='update_batch_loc_to_station')
        self.machine.add_transition('process_state_transitions', source='added_batch_update',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_batches_loaded'])

        # enable autofunctions
        self.machine.add_transition('process_state_transitions',source='close_chemspeed_door',dest='enable_auto_functions', conditions='is_station_job_ready')

        # process batch after closing door and enabling autofunctions
        self.machine.add_transition('process_state_transitions', source='enable_auto_functions',dest='retreat_from_chemspeed', unless='is_station_operation_complete', conditions='is_station_job_ready')
        
        self.machine.add_transition('process_state_transitions', source='retreat_from_chemspeed',dest='chemspeed_process', conditions='is_station_job_ready')

        # navigate to the chemspeed
        self.machine.add_transition('process_state_transitions', source='chemspeed_process',dest='disable_auto_functions', conditions='is_station_job_ready', before='process_batches')

        # unload after openning door
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='unload_batch', conditions=['is_station_operation_complete','is_station_job_ready'])
        # self.machine.add_transition('process_state_transitions', source='unload_batch',dest='=', unless='are_all_batches_unloaded', conditions='is_station_job_ready', before='update_batch_loc_to_robot')
        self.machine.add_transition('process_state_transitions', source='unload_batch',dest='removed_batch_update', conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions', source='removed_batch_update',dest='unload_batch', unless='are_all_batches_unloaded', conditions='is_station_job_ready')

        # close door after unloading
        # self.machine.add_transition('process_state_transitions', source='unload_batch',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_batches_unloaded'], before='update_batch_loc_to_robot')
        self.machine.add_transition('process_state_transitions', source='removed_batch_update',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_batches_unloaded'])
        
        # complete
        self.machine.add_transition('process_state_transitions', source='enable_auto_functions',dest='final_state', conditions=['is_station_job_ready','is_station_operation_complete'])

    def is_station_operation_complete(self):
        return self.operation_complete

    def request_open_door(self):
        self._station.assign_station_op(CSOpenDoorOpDescriptor.from_args())

    def request_close_door(self):
        self._station.assign_station_op(CSCloseDoorOpDescriptor.from_args())

    def request_load_batch(self):
        robot_job = KukaLBRTask.from_args(name='LoadChemSpeed',params=[True,self._current_batch_index+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)

    def request_unload_batch(self):
        robot_job = KukaLBRTask.from_args(name='UnloadChemSpeed',params=[False,self._current_batch_index+1],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_batch_index].id
        self._station.request_robot_op(robot_job,current_batch_id)
        

    def request_navigate_to_chemspeed(self):
        self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO get this property from the config

    def request_disable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))

    def request_process_operation(self):
        current_op_dict = self._station.assigned_batches[-1].recipe.get_current_task_op_dict()
        current_op = StationFactory.create_op_from_dict(current_op_dict)
        if isinstance (current_op,CSCSVJobOpDescriptor):
            contacnated_csv = ''
            for batch in self._station.assigned_batches:
                current_op_dict = batch.recipe.get_current_task_op_dict()
                current_op = StationFactory.create_op_from_dict(current_op_dict)
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
        self.operation_complete = True

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self.operation_complete = False
        self.to_init_state()

    def _print_state(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')



