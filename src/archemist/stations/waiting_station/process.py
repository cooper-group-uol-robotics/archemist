
from transitions import State
from archemist.core.state.robot_op import DropBatchOpDescriptor, CollectBatchOpDescriptor
from .state import WaitOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.persistence.models_proxy import ModelProxy
from typing import Union

class WaitingStationProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'),
            State(name='waiting_process', on_enter=['request_process_operation']),
            State(name='load_lot', on_enter=['request_load_lot']),
            State(name='unload_lot', on_enter=['request_unload_lot']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'load_lot'},
            {'source':'load_lot','dest':'waiting_process', 'conditions':'are_req_robot_ops_completed'},
            {'source':'waiting_process','dest':'unload_lot', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_lot','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]

    ''' states callbacks '''

    def request_load_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.lot_slot*num_batches
        
        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["place_batch_index"] = batches_offset + index + 1
            params_dict["perform_6p_calib"] = False
            robot_op = DropBatchOpDescriptor.from_args(name='LoadWaitingStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch)
            req_robot_ops.append(robot_op)
        
        self.request_robot_ops(req_robot_ops)

    def request_unload_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.lot_slot*num_batches
        
        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["perform_6p_calib"] = False
            params_dict["pick_batch_index"] = batches_offset + index + 1
            robot_op = CollectBatchOpDescriptor.from_args(name='UnloadWaitingStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch)
            req_robot_ops.append(robot_op)
        
        self.request_robot_ops(req_robot_ops)

    def request_process_operation(self):
        current_op: WaitOp  = self.generate_operation("wait_for", target_lot=self.lot)
        self.request_station_op(current_op)


