from transitions import Machine, State
from archemist.state.station import Station, StationState
from archemist.state.robot import RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.state.stations.chemspeed_flex_station import CSCloseDoorOpDescriptor, CSOpenDoorOpDescriptor, CSJobOutputDescriptor
from archemist.util import Location
import archemist.persistence.objectConstructor

class ChemSpeedRackSm():
    
    def __init__(self, station: Station, args: dict):

        self._station = station
        self.batch_mode = args['batch_mode']
        self.operation_complete = False
        self._loaded_racks = 0
        self._current_rack_index = -1

        #self._load_frame = args['load_frame']
        #self._racks_no = args['no_racks']
        #self._current_rack = 0


        

        ''' States '''
        states = [ State(name='init_state', on_enter='_print_state'), 
            State(name='open_chemspeed_door', on_enter=['request_open_door', '_print_state']),
            State(name='disable_auto_functions', on_enter=['request_disable_auto_functions', '_print_state']),
            State(name='navigate_to_chemspeed', on_enter=['request_navigate_to_chemspeed', '_print_state']),
            State(name='retreat_from_chemspeed', on_enter=['request_navigate_to_chemspeed', '_print_state']),
            State(name='enable_auto_functions', on_enter=['request_enable_auto_functions', '_print_state']), 
            State(name='chemspeed_process', on_enter=['request_process_operation', '_print_state']),
            State(name='load_rack', on_enter=['request_load_rack', '_print_state']),
            State(name='close_chemspeed_door', on_enter=['request_close_door', '_print_state']), 
            State(name='unload_rack', on_enter=['request_unload_rack', '_print_state']),
            State(name='final_state', on_enter=['finalize_batch_processing', '_print_state'])]
        
        self.machine = Machine(self, states=states, initial='init_state')

        ''' Transitions '''

        # init_state transitions
        self.machine.add_transition('process_state_transitions',source='init_state',dest='disable_auto_functions', conditions='all_batches_assigned')

        # disable autofunctions and navigate to the station
        self.machine.add_transition('process_state_transitions',source='disable_auto_functions',dest='navigate_to_chemspeed', conditions='is_station_job_ready')

        # open chemspeed door
        self.machine.add_transition('process_state_transitions',source='navigate_to_chemspeed',dest='open_chemspeed_door', conditions='is_station_job_ready')

        # load all racks into the chemspeed
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='load_rack', unless='is_station_operation_complete' ,conditions='is_station_job_ready')
        self.machine.add_transition('process_state_transitions', source='load_rack',dest='=', unless='are_all_racks_loaded', conditions='is_station_job_ready', before='update_batch_loc_to_station')

        # close door after loading rack
        self.machine.add_transition('process_state_transitions', source='load_rack',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_racks_loaded'], before='update_batch_loc_to_station')

        # enable autofunctions
        self.machine.add_transition('process_state_transitions',source='close_chemspeed_door',dest='enable_auto_functions', conditions='is_station_job_ready')

        # process batch after closing door and enabling autofunctions
        self.machine.add_transition('process_state_transitions', source='enable_auto_functions',dest='retreat_from_chemspeed', unless='is_station_operation_complete', conditions='is_station_job_ready')
        
        self.machine.add_transition('process_state_transitions', source='retreat_from_chemspeed',dest='chemspeed_process', conditions='is_station_job_ready')

        # navigate to the chemspeed
        self.machine.add_transition('process_state_transitions', source='chemspeed_process',dest='disable_auto_functions', conditions='is_station_job_ready', before='process_batches')

        # unload after openning door
        self.machine.add_transition('process_state_transitions', source='open_chemspeed_door',dest='unload_rack', conditions=['is_station_operation_complete','is_station_job_ready'])
        self.machine.add_transition('process_state_transitions', source='unload_rack',dest='=', unless='are_all_racks_unloaded', conditions='is_station_job_ready', before='update_batch_loc_to_robot')

        # close door after unloading
        self.machine.add_transition('process_state_transitions', source='unload_rack',dest='close_chemspeed_door', conditions=['is_station_job_ready','are_all_racks_unloaded'], before='update_batch_loc_to_robot')
        
        # complete
        self.machine.add_transition('process_state_transitions', source='enable_auto_functions',dest='final_state', conditions=['is_station_job_ready','is_station_operation_complete'])



    def is_station_job_ready(self):
        return not self._station.has_station_op() and not self._station.has_robot_job()

    def is_station_operation_complete(self):
        return self.operation_complete

    def is_rack_loaded(self):
        return self.rack_loaded

    def all_batches_assigned(self):
        return not self._station.has_free_batch_capacity()

    def update_batch_loc_to_station(self):
        self._station.assigned_batches[self._current_rack_index].location = self._station.location

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batches[self._current_rack_index].location = Location(-1,-1,f'{last_executed_robot_op.output.executing_robot}/Deck')

    def are_all_racks_loaded(self):
        return self._loaded_racks == self._station.batch_capacity

    def are_all_racks_unloaded(self):
        return self._loaded_racks == 0

    def request_open_door(self):
        self._station.set_station_op(CSOpenDoorOpDescriptor(dict(), CSJobOutputDescriptor()))

    def request_close_door(self):
        self._station.set_station_op(CSCloseDoorOpDescriptor(dict(), CSJobOutputDescriptor()))

    def request_load_rack(self):
        self._current_rack_index += 1
        self._loaded_racks += 1
        robot_job = KukaLBRTask('LoadChemSpeed',[True,self._current_rack_index+1], RobotTaskType.UNLOAD_FROM_ROBOT, self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_rack_index].id
        self._station.set_robot_job(robot_job,current_batch_id)

    def request_unload_rack(self):
        robot_job = KukaLBRTask('UnloadChemSpeed',[False,self._current_rack_index+1],RobotTaskType.LOAD_TO_ROBOT, self._station.location)
        current_batch_id = self._station.assigned_batches[self._current_rack_index].id
        self._station.set_robot_job(robot_job,current_batch_id)
        self._current_rack_index -= 1
        self._loaded_racks -= 1
        

    def request_navigate_to_chemspeed(self):
        self._station.set_robot_job(KukaNAVTask(Location(26,1,''), True)) #TODO get this property from the config

    def request_disable_auto_functions(self):
        self._station.set_robot_job(KukaLBRMaintenanceTask('DiableAutoFunctions',[False]))

    def request_enable_auto_functions(self):
        self._station.set_robot_job(KukaLBRMaintenanceTask('EnableAutoFunctions',[False]))

    def request_process_operation(self):
        current_op_dict = self._station.assigned_batches[-1].recipe.get_current_task_op_dict()
        current_op = archemist.persistence.objectConstructor.ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        self._station.set_station_op(current_op)

    def process_batches(self):
        last_operation_op = self._station.station_op_history[-1]
        #TODO if we have multiple racks loop through them
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



