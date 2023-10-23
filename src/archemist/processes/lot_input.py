from transitions import State
from typing import List, Dict, Any

from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.robot_op import RobotTaskOpDescriptor, RobotTaskType

class BasicLotInputProcess(StationProcess):

    def __init__(self, process_model): 
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='pickup_lot', on_enter=['request_pickup_lot']),
            State(name='final_state')]
        
        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'pickup_lot'},
            {'source':'pickup_lot','dest':'final_state', 'conditions':'are_req_robot_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot, key_op_dicts_list: List[Dict[str, Any]] = None, **kwargs):
        model = StationProcessModel()
        cls._set_model_common_fields(model, "", lot, key_op_dicts_list, **kwargs)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        pass

    def request_pickup_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.processing_slot*num_batches
        
        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["pick_batch_index"] = batches_offset + index + 1
            params_dict["perform_6p_calib"] = False
            params_dict["place_batch_index"] = index + 1
            robot_op = RobotTaskOpDescriptor.from_args(name='PickupInputRack', target_robot="MobileRobot", type=RobotTaskType.LOAD_TO_ROBOT,
                                                       params=params_dict, related_batch=batch)
            req_robot_ops.append(robot_op)
        
        self.request_robot_ops(req_robot_ops)