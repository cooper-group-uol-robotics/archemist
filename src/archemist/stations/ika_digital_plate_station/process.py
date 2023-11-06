from transitions import State
from archemist.core.state.lot import Lot
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import DropBatchOpDescriptor, RobotTaskOpDescriptor
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, Dict, Any, List


class PXRDWorkflowStirringProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 
            State(name='place_lot', on_enter=['request_place_lot']),
            State(name='load_stir_plate', on_enter=['request_load_stir_plate']),
            State(name='stir', on_enter=['request_stir_op']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'place_lot'},
            {'source':'place_lot', 'dest': 'load_stir_plate', 'conditions':'are_req_robot_ops_completed'},
            {'source':'load_stir_plate','dest':'stir', 'conditions':'are_req_robot_ops_completed'},
            {'source':'stir','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
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
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     "Station",
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["eight_well_rack_first"] = eight_well_rack_first
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_place_lot(self):
        # place the 8-well rack
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = True
        robot_op_1 = DropBatchOpDescriptor.from_args(name='LoadEightWRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_1)
        # place the pxrd rack
        batch_2 = self.lot.batches[self.pxrd_batch_index]
        params_dict["perform_6p_calib"] = False
        robot_op_2 = DropBatchOpDescriptor.from_args(name='LoadPXRDRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_2)
        
        self.request_robot_ops([robot_op_1, robot_op_2])

    def request_load_stir_plate(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        robot_op = RobotTaskOpDescriptor.from_args(name='loadIKAPlate', target_robot="YuMiRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_stir_op(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        current_op = self.generate_operation("stir_op", target_batch=batch_1)
        self.request_station_op(current_op)
