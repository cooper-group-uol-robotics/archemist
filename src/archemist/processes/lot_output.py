from transitions import State
from typing import List, Dict, Any

from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess
from archemist.core.state.robot_op import DropBatchOpDescriptor

class BasicLotOutputProcess(StationProcess):

    def __init__(self, process_model): 
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='place_lot', on_enter=['request_place_lot']),
            State(name='final_state')]
        
        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'place_lot'},
            {'source':'place_lot','dest':'final_state', 'conditions':'are_req_robot_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  operations: List[Dict[str, Any]] = None,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        return super().from_args(lot, operations, skip_robot_ops, skip_station_ops, skip_ext_procs)

    ''' states callbacks '''

    def initialise_process_data(self):
        pass

    def request_place_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.lot_slot*num_batches
        
        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["perform_6p_calib"] = False
            params_dict["place_batch_index"] = batches_offset + index + 1
            robot_op = DropBatchOpDescriptor.from_args(name='PlaceRack', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch)
            req_robot_ops.append(robot_op)
        
        self.request_robot_ops(req_robot_ops)