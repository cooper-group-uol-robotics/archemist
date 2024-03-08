from transitions import State
from .state import ChemSpeedFlexStation
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import (CollectBatchOp,
                                           DropBatchOp,
                                           RobotNavOp,
                                           RobotWaitOp)
from .state import CSCloseDoorOp, CSOpenDoorOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any


class CMFlexLiquidDispenseProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'),
                       State(name='prep_state',
                             on_enter='initialise_process_data'),
                       State(name='open_chemspeed_door',
                             on_enter=['request_open_door']),
                       State(name='navigate_to_chemspeed', on_enter=[
                           'request_navigate_to_chemspeed']),
                       State(name='retreat_from_chemspeed', on_enter=[
                           'request_retreat_from_chemspeed']),
                       State(name='chemspeed_process', on_enter=[
                           'request_process_operation']),
                       State(name='load_lot', on_enter=['request_load_lot']),
                       State(name='close_chemspeed_door',
                             on_enter=['request_close_door']),
                       State(name='unload_lot', on_enter=[
                             'request_unload_lot']),
                       State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source': 'init_state', 'dest': 'prep_state'},
            {'source': 'prep_state', 'dest': 'navigate_to_chemspeed'},
            {'source': 'navigate_to_chemspeed', 'dest': 'open_chemspeed_door',
                'conditions': 'are_req_robot_ops_completed'},
            {'source': 'open_chemspeed_door', 'dest': 'load_lot',
                'unless': 'is_dispense_finished', 'conditions': 'are_req_station_ops_completed'},
            {'source': 'load_lot', 'dest': 'close_chemspeed_door',
                'conditions': 'are_req_robot_ops_completed'},
            {'source': 'close_chemspeed_door', 'dest': 'retreat_from_chemspeed',
                'conditions': 'are_req_station_ops_completed'},
            {'source': 'retreat_from_chemspeed', 'dest': 'chemspeed_process',
                'unless': 'is_dispense_finished', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'chemspeed_process', 'dest': 'navigate_to_chemspeed',
                'conditions': 'are_req_station_ops_completed'},
            {'source': 'open_chemspeed_door', 'dest': 'unload_lot', 'conditions': [
                'is_dispense_finished', 'are_req_station_ops_completed']},
            {'source': 'unload_lot', 'dest': 'close_chemspeed_door',
                'conditions': 'are_req_robot_ops_completed'},
            {'source': 'retreat_from_chemspeed', 'dest': 'final_state', 'conditions': [
                'are_req_robot_ops_completed', 'is_dispense_finished']}
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
                                     ChemSpeedFlexStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['dispense_finished'] = False

    def request_open_door(self):
        station_op = CSOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_close_door(self):
        station_op = CSCloseDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_lot(self):
        req_robot_ops = []
        num_batches = self.lot.num_batches
        batches_offset = self.lot_slot*num_batches

        for index, batch in enumerate(self.lot.batches):
            params_dict = {}
            params_dict["place_batch_index"] = batches_offset + index + 1
            params_dict["perform_6p_calib"] = False
            robot_op = DropBatchOp.from_args(name='LoadChemSpeed', target_robot="KMRIIWARobot",
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
            robot_op = CollectBatchOp.from_args(name='UnloadChemSpeed', target_robot="KMRIIWARobot",
                                                params=params_dict, target_batch=batch)
            req_robot_ops.append(robot_op)

        self.request_robot_ops(req_robot_ops)

    def request_navigate_to_chemspeed(self):
        nav_to_station = RobotNavOp.from_args(name="nav_to_chemspeed",
                                              target_robot="KMRIIWARobot",
                                              target_location=None)
        wait_for_next_op = RobotWaitOp.from_args("KMRIIWARobot", 3)

        self.request_robot_ops([nav_to_station, wait_for_next_op])

    def request_retreat_from_chemspeed(self):
        nav_away = RobotNavOp.from_args(name="retreat_from_chemspeed",
                                        target_robot="KMRIIWARobot",
                                        target_location=None)

        self.request_robot_ops([nav_away])

    def request_process_operation(self):
        current_op = self.generate_operation(
            "dispense_op", target_lot=self.lot)
        self.data['dispense_finished'] = True
        self.request_station_op(current_op)

    ''' transition callbacks '''

    def is_dispense_finished(self):
        return self.data['dispense_finished']
