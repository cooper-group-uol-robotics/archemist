from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.panda_robot.state import PandaRobotTask
from archemist.core.state.station_process import StationProcess, StationProcessData
from .state import  QuantosSolidDispenserQB1, DispenseOpDescriptor, OpenDoorOpDescriptor, CloseDoorOpDescriptor, UnlockCartridgeOpDescriptor


class QuantosQb1StationProcess(StationProcess):
    
    def __init__(self, station:  QuantosSolidDispenserQB1, process_data: StationProcessData, **kwargs): 
        
        ''' States '''
        states = [State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='open_quantos_door', on_enter= 'request_open_quantos_door'),
            State(name='load_vial', on_enter= 'request_load_sample_job'),
            State(name='load_cartridge', on_enter= 'request_load_cartridge_job'),
            State(name='close_quantos_door', on_enter= 'request_close_quantos_door'),
            State(name='quantos_dispense', on_enter= 'request_quantos_dispense'),
            State(name='open_quantos_door', on_enter= 'request_open_quantos_door'),
            State(name='unload_vial', on_enter='request_unload_sample_job'),
            State(name='unlock_cartridge', on_enter='request_unlock_cartridge'),
            State(name= 'unload_cartridge', on_enter= 'request_unload_cartridge_job'),
            State(name='final_state', on_enter='finalize_batch_processing')]

        ''' Transitions '''
        transitions = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state', 'dest': 'open_quantos_door'},
            {'source': 'open_quantos_door', 'dest': 'load_vial', 'unless': 'is_station_operation_complete', 'conditions':'are_req_station_ops_completed'},
            {'source':'load_vial','dest':'load_cartridge', 'conditions':'are_req_robot_ops_completed'},
            {'source':'load_cartridge','dest':'close_quantos_door', 'conditions':'are_req_robot_ops_completed', 'after': 'db_load_cartridge'},
            {'source':'close_quantos_door','dest':'quantos_dispense', 'conditions':'are_req_station_ops_completed'},
            {'source':'quantos_dispense','dest':'open_quantos_door', 'conditions': 'are_req_station_ops_completed', 'before': 'process_sample'},
            {'source':'open_quantos_door','dest':'unload_vial', 'conditions':['are_req_station_ops_completed', 'is_station_operation_complete']},
            {'source':'unload_vial','dest':'unlock_cartridge', 'conditions':'are_req_robot_ops_completed'},
            {'source': 'unlock_cartridge', "dest": "unload_cartridge", "conditions": "are_req_station_ops_completed"},
            {'source':'unload_cartridge','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]
        super().__init__(station, process_data, states, transitions)

    ''' states callbacks '''

    def initialise_process_data(self):
        """Set variables needed on entering the station"""
        self._process_data.status['operation_complete'] = False
      


    def request_load_sample_job(self):
        """Request panda load vial to the quantos"""
        #robot_job = PandaRobotTask.from_args(name='PresentVial', #TODO accept variables to accomodate multiple vials
        #                                  params=[], 
        #                                type=RobotTaskType.MANIPULATION, location=self._station.location)
        #current_batch_id = self._process_data.batches[0].id
        #self.request_robot_op(robot_job,current_batch_id)
        print('request load sample job')


    def request_unload_sample_job(self):
        """Request panda unload the vial from the quantos"""
        #robot_job = PandaRobotTask.from_args(name='ReturnVial', #TODO accept variables to accomodate multiple vials
        #                                  params=[],
        #                                type=RobotTaskType.MANIPULATION, location=self._station.location)
        #current_batch_id = self._process_data.batches[0].id
        #self.request_robot_op(robot_job,current_batch_id)
        print('request unload sample job')

    def request_load_cartridge_job(self):
        """Request panda load the cartridge to the quantos"""
        #robot_job = PandaRobotTask.from_args(name = "PresentCartridge",
        #                                     params = [],
        #                                     type=RobotTaskType.MANIPULATION, location=self._station.location)
        #current_batch_id = self._process_data.batches[0].id
        #self.request_robot_op(robot_job,current_batch_id)
        print('request load cartridge job')
        self.db_load_cartridge()
    



    def request_unload_cartridge_job(self):
        """Request panda unload the cartridge from the quantos"""
        
       #robot_job = PandaRobotTask.from_args(name = "ReturnCartridge",
       #                                     params = [],
       #                                     type=RobotTaskType.MANIPULATION, location=self._station.location)
       #current_batch_id = self._process_data.batches[0].id
       #self.request_robot_op(robot_job,current_batch_id)
        print('request unload cartridge job')

    def request_quantos_dispense(self):
        #the task operaiton is specified in the recipe so can be wrapped like this
        current_batch = self._process_data.batches[0]
        attached_recipe = current_batch.recipe
        station_op = attached_recipe.get_current_task_op()
        
        self.request_station_op(station_op)
   
    def request_open_quantos_door(self):
        #these task operations are not called explicitly in the recipe and so need to be created.

        current_op = OpenDoorOpDescriptor.from_args()
        self.request_station_op(current_op)

    def request_close_quantos_door(self):
        #these task operations are not called explicitly in the recipe and so need to be created.

        current_op = CloseDoorOpDescriptor.from_args()
        self.request_station_op(current_op)

    def request_unlock_cartridge(self):
        """unlocks the quantos cartridge"""
        current_op = UnlockCartridgeOpDescriptor.from_args()
        self.request_station_op(current_op)

    def finalize_batch_processing(self):
        #these task operations are not called explicitly in the recipe and so need to be created.

        for batch in self._process_data.batches:
            self._station.process_assinged_batch(batch)
        
    ''' states callbacks '''
    
    def process_sample(self):
        #this step is needed to add the completed station operation to the sample history
   
        last_operation_op_uuid = self._process_data.station_ops_history[-1]
        last_operation_op = self._station.completed_station_ops[last_operation_op_uuid]
        current_batch = self._process_data.batches[0]
        current_batch.add_station_op_to_current_sample(last_operation_op)
        current_batch.process_current_sample()
        self._process_data.status['operation_complete'] = True


    def db_load_cartridge(self):
        
        current_batch = self._process_data.batches[0]
        recipe = current_batch.recipe
        solid = recipe.solids[0]["name"] #This will not work if more than one solid is specified
      

        id = self._station.get_cartridge_id(solid)
        
        print(id)
    
        
        self._station.load_cartridge(id)


        
    def is_station_operation_complete(self):
            return self._process_data.status['operation_complete']


        

