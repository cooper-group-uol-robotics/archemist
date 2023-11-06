from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.robot_op import (RobotTaskOpDescriptor,
                                           DropBatchOpDescriptor,
                                           CollectBatchOpDescriptor,
                                           RobotWaitOpDescriptor)
from .state import PXRDOpenDoorOp, PXRDCloseDoorOp
from archemist.core.util import Location
from typing import Union
from typing import List, Dict, Any

class PXRDWorkflowAnalysisProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [ State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='open_pxrd_door', on_enter='request_open_pxrd_door'),
            State(name='open_pxrd_door_update', on_enter='request_open_pxrd_door_update'),
            State(name='pxrd_process', on_enter='request_pxrd_process'),
            State(name='load_pxrd', on_enter='request_load_pxrd'),
            State(name='close_pxrd_door', on_enter='request_close_pxrd_door'),
            State(name='close_pxrd_door_update', on_enter='request_close_pxrd_door_update'),
            State(name='unload_pxrd', on_enter='request_unload_pxrd'),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'open_pxrd_door'},
            {'source':'open_pxrd_door','dest':'open_pxrd_door_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'open_pxrd_door_update','dest':'load_pxrd', 'unless':'is_batch_analysed' ,'conditions':'are_req_station_ops_completed'},
            {'source':'load_pxrd','dest':'close_pxrd_door', 'conditions':'are_req_robot_ops_completed'},
            {'source':'close_pxrd_door','dest':'close_pxrd_door_update', 'conditions':'are_req_robot_ops_completed'}, 
            {'source':'close_pxrd_door_update','dest':'pxrd_process', 'unless':'is_batch_analysed', 'conditions':'are_req_station_ops_completed'},           
            {'source':'pxrd_process','dest':'open_pxrd_door', 'conditions':'are_req_station_ops_completed'},
            {'source':'open_pxrd_door','dest':'open_pxrd_door_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'open_pxrd_door_update','dest':'unload_pxrd', 'conditions':['is_batch_analysed','are_req_station_ops_completed']},
            {'source':'unload_pxrd','dest':'close_pxrd_door', 'conditions':'are_req_robot_ops_completed'},
            {'source':'close_pxrd_door','dest':'close_pxrd_door_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'close_pxrd_door_update','dest':'final_state', 'conditions':['are_req_station_ops_completed','is_batch_analysed']}
        ]

        if self.data["eight_well_rack_first"]:
            self.pxrd_batch_index = 1
        else:
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
    def initialise_process_data(self):
        self.data['batch_analysed'] = False

    def request_open_pxrd_door(self):
        door_loc = Location.from_args(coordinates=(19, 1), descriptor="PXRDDoorLocation")
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOpDescriptor.from_args(name="OpenDoors", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_location=door_loc)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def request_open_pxrd_door_update(self):
        station_op = PXRDOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_close_pxrd_door(self):
        door_loc = Location.from_args(coordinates=(19, 1), descriptor="PXRDDoorLocation")
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOpDescriptor.from_args(name="CloseDoors", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_location=door_loc)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def request_close_pxrd_door_update(self):
        station_op = PXRDCloseDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_pxrd(self):
        target_batch = self.lot.batches[self.pxrd_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        load_pxrd_robot_op = DropBatchOpDescriptor.from_args(name="LoadPXRD", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([load_pxrd_robot_op, robot_wait_op])

    def request_unload_pxrd(self):
        target_batch = self.lot.batches[self.pxrd_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        unload_pxrd_robot_op = CollectBatchOpDescriptor.from_args(name="UnloadPXRD", target_robot="KMRIIWARobot",
                                                   params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="KMRIIWARobot", timeout=5)
        self.request_robot_ops([unload_pxrd_robot_op, robot_wait_op])

    def request_pxrd_process(self):
        batch = self.lot.batches[self.pxrd_batch_index]
        current_op = self.generate_operation("analyse_op", target_batch=batch)
        self.request_station_op(current_op)
        self.data['batch_analysed'] = True

    ''' transition callbacks'''
    def is_batch_analysed(self):
        return self.data['batch_analysed']


