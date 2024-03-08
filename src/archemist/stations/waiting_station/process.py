
from transitions import State
from archemist.core.state.robot_op import DropBatchOp, CollectBatchOp
from .state import WaitOp, WaitingStation
from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.persistence.models_proxy import ModelProxy
from typing import Union, List, Dict, Any


class WaitingStationProcess(StationProcess):

    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'),
                       State(name='prep_state'),
                       State(name='waiting_process', on_enter=[
                             'request_process_operation']),
                       State(name='load_lot', on_enter=['request_load_lot']),
                       State(name='unload_lot', on_enter=[
                             'request_unload_lot']),
                       State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source': 'init_state', 'dest': 'prep_state'},
            {'source': 'prep_state', 'dest': 'load_lot'},
            {'source': 'load_lot', 'dest': 'waiting_process',
                'conditions': 'are_req_robot_ops_completed'},
            {'source': 'waiting_process', 'dest': 'unload_lot',
                'conditions': 'are_req_station_ops_completed'},
            {'source': 'unload_lot', 'dest': 'final_state',
                'conditions': 'are_req_robot_ops_completed'}
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool = False,
                  skip_robot_ops: bool = False,
                  skip_station_ops: bool = False,
                  skip_ext_procs: bool = False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     WaitingStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_load_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.lot_slot*num_batches

        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["place_batch_index"] = batches_offset + index + 1
            params_dict["perform_6p_calib"] = False
            robot_op = DropBatchOp.from_args(name='LoadWaitingStation', target_robot="KMRIIWARobot",
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
            robot_op = CollectBatchOp.from_args(name='UnloadWaitingStation', target_robot="KMRIIWARobot",
                                                params=params_dict, target_batch=batch)
            req_robot_ops.append(robot_op)

        self.request_robot_ops(req_robot_ops)

    def request_process_operation(self):
        current_op: WaitOp = self.generate_operation(
            "wait_op", target_lot=self.lot)
        self.request_station_op(current_op)
