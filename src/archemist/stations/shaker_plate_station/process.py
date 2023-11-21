from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from .state import ShakerPlateStation
from archemist.core.state.robot_op import RobotTaskOp, CollectBatchOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import List, Dict, Any

from typing import Union

class PXRDWorkflowShakingProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'),
            State(name='load_shaker_plate', on_enter='request_load_shaker_plate'),
            State(name='shake', on_enter='request_shake_op'),
            State(name='unload_shaker_plate', on_enter='request_unload_shaker_plate'),
            State(name='unscrew_caps', on_enter='request_unscrew_caps'),
            State(name='pick_lot', on_enter='request_lot_pickup'),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'load_shaker_plate'},
            {'source':'load_shaker_plate','dest':'shake', 'conditions':'are_req_robot_ops_completed'},
            {'source':'shake','dest':'unload_shaker_plate', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_shaker_plate','dest':'unscrew_caps', 'conditions':'are_req_robot_ops_completed'},
            {'source':'unscrew_caps','dest':'pick_lot', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pick_lot','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]

        if self.data["eight_well_rack_first"]:
            self.eight_well_batch_index = 0
            self.pxrd_batch_index = 1
        else:
            self.eight_well_batch_index = 1
            self.pxrd_batch_index = 0

    @classmethod
    def from_args(cls, lot: Lot,
                  eight_well_rack_first: bool,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     ShakerPlateStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["eight_well_rack_first"] = eight_well_rack_first
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_load_shaker_plate(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        robot_op = RobotTaskOp.from_args(name='invertAndLoadShakerPlate', target_robot="YuMiRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_shake_op(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        current_op = self.generate_operation("shake_op", target_batch=batch_1)
        self.request_station_op(current_op)

    def request_unload_shaker_plate(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        robot_op = RobotTaskOp.from_args(name='invertAndLoadWellPlate', target_robot="YuMiRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_unscrew_caps(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        robot_op = RobotTaskOp.from_args(name='unscrewCaps', target_robot="YuMiRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_lot_pickup(self):
        # pickup the 8-well rack
        params_dict = {}
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        params_dict["perform_6p_calib"] = False
        robot_op_1 = CollectBatchOp.from_args(name='UnloadEightWRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_1)

        # pickup the pxrd rack
        batch_2 = self.lot.batches[self.pxrd_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = True
        robot_op_2 = CollectBatchOp.from_args(name='UnloadPXRDRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_2)
        
        self.request_robot_ops([robot_op_1, robot_op_2])
