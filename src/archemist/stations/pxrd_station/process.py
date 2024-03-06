from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.robot_op import (RobotTaskOp,
                                           DropBatchOp,
                                           CollectBatchOp,
                                           RobotWaitOp)
from .state import PXRDStation
from typing import Union
from typing import List, Dict, Any


class PXRDWorkflowAnalysisProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'),
                       State(name='prep_state', on_enter='initialise_process_data'),
                       State(name='open_pxrd_door', on_enter='request_open_pxrd_door'),
                       State(name='open_pxrd_door_update', on_enter='change_pxrd_door_to_open'),
                       State(name='pxrd_process', on_enter='request_pxrd_process'),
                       State(name='load_pxrd', on_enter='request_load_pxrd'),
                       State(name='close_pxrd_door', on_enter='request_close_pxrd_door'),
                       State(name='close_pxrd_door_update', on_enter='change_pxrd_door_to_closed'),
                       State(name='unload_pxrd', on_enter='request_unload_pxrd'),
                       State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source': 'init_state', 'dest': 'prep_state'},
            {'source': 'prep_state', 'dest': 'open_pxrd_door'},
            {'source': 'open_pxrd_door', 'dest': 'open_pxrd_door_update', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'open_pxrd_door_update', 'dest': 'load_pxrd', 'unless': 'is_batch_analysed'},
            {'source': 'load_pxrd', 'dest': 'close_pxrd_door', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'close_pxrd_door', 'dest': 'close_pxrd_door_update', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'close_pxrd_door_update', 'dest': 'pxrd_process', 'unless': 'is_batch_analysed'},
            {'source': 'pxrd_process', 'dest': 'open_pxrd_door', 'conditions': 'are_req_station_ops_completed'},
            {'source': 'open_pxrd_door', 'dest': 'open_pxrd_door_update', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'open_pxrd_door_update', 'dest': 'unload_pxrd', 'conditions': 'is_batch_analysed'},
            {'source': 'unload_pxrd', 'dest': 'close_pxrd_door', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'close_pxrd_door', 'dest': 'close_pxrd_door_update', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'close_pxrd_door_update', 'dest': 'final_state', 'conditions': 'is_batch_analysed'}
        ]

        if self.data["eight_well_rack_first"]:
            self.pxrd_batch_index = 1
        else:
            self.pxrd_batch_index = 0

    @classmethod
    def from_args(cls, lot: Lot,
                  eight_well_rack_first: bool,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool = False,
                  skip_robot_ops: bool = False,
                  skip_station_ops: bool = False,
                  skip_ext_procs: bool = False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     PXRDStation.__name__,
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

    def initialise_process_data(self):
        self.data['batch_analysed'] = False

    def request_open_pxrd_door(self):
        pxrd_station: PXRDStation = self.get_assigned_station()
        door_loc = pxrd_station.doors_location
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOp.from_args(name="OpenDoors", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_location=door_loc)
        robot_wait_op = RobotWaitOp.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def change_pxrd_door_to_open(self):
        pxrd_station: PXRDStation = self.get_assigned_station()
        pxrd_station.door_closed = False

    def request_close_pxrd_door(self):
        pxrd_station: PXRDStation = self.get_assigned_station()
        door_loc = pxrd_station.doors_location
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOp.from_args(name="CloseDoors", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_location=door_loc)
        robot_wait_op = RobotWaitOp.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def change_pxrd_door_to_closed(self):
        pxrd_station: PXRDStation = self.get_assigned_station()
        pxrd_station.door_closed = True

    def request_load_pxrd(self):
        target_batch = self.lot.batches[self.pxrd_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        load_pxrd_robot_op = DropBatchOp.from_args(name="LoadPXRD", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOp.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([load_pxrd_robot_op, robot_wait_op])

    def request_unload_pxrd(self):
        target_batch = self.lot.batches[self.pxrd_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        unload_pxrd_robot_op = CollectBatchOp.from_args(name="UnloadPXRD", target_robot="KMRIIWARobot",
                                                        params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOp.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([unload_pxrd_robot_op, robot_wait_op])

    def request_pxrd_process(self):
        batch = self.lot.batches[self.pxrd_batch_index]
        current_op = self.generate_operation("analyse_op", target_batch=batch)
        self.request_station_op(current_op)
        self.data['batch_analysed'] = True

    ''' transition callbacks'''

    def is_batch_analysed(self):
        return self.data['batch_analysed']
