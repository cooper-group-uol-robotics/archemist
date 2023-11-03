from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.robot_op import (RobotTaskOpDescriptor,
                                           DropBatchOpDescriptor,
                                           CollectBatchOpDescriptor,
                                           RobotWaitOpDescriptor)
from .state import PXRDAnalysisOp, PXRDOpenDoorOp, PXRDCloseDoorOp
from archemist.core.util import Location
from typing import Union

class PXRDProcess(StationProcess):
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

    ''' states callbacks '''
    def initialise_process_data(self):
        self.data['batch_analysed'] = False

    def request_open_pxrd_door(self):
        door_loc = Location.from_args(coordinates=(19, 1), descriptor="PXRDDoorLocation")
        target_batch = self.lot.batches[0]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOpDescriptor.from_args(name="OpenDoors", target_robot="MobileRobot",
                                                   params=params_dict, target_location=door_loc,
                                                   target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="MobileRobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def request_open_pxrd_door_update(self):
        station_op = PXRDOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_close_pxrd_door(self):
        door_loc = Location.from_args(coordinates=(19, 1), descriptor="PXRDDoorLocation")
        target_batch = self.lot.batches[0]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        open_door_robot_op = RobotTaskOpDescriptor.from_args(name="CloseDoors", target_robot="MobileRobot",
                                                   params=params_dict, target_location=door_loc,
                                                   target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="MobileRobot", timeout=5)
        self.request_robot_ops([open_door_robot_op, robot_wait_op])

    def request_close_pxrd_door_update(self):
        station_op = PXRDCloseDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_pxrd(self):
        target_batch = self.lot.batches[0]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        load_pxrd_robot_op = DropBatchOpDescriptor.from_args(name="LoadPXRD", target_robot="MobileRobot",
                                                   params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="MobileRobot", timeout=5)
        self.request_robot_ops([load_pxrd_robot_op, robot_wait_op])

    def request_unload_pxrd(self):
        target_batch = self.lot.batches[0]
        params_dict = {}
        params_dict["perform_6p_calib"] = False
        unload_pxrd_robot_op = CollectBatchOpDescriptor.from_args(name="UnloadPXRD", target_robot="MobileRobot",
                                                   params=params_dict, target_batch=target_batch)
        robot_wait_op = RobotWaitOpDescriptor.from_args(target_robot="MobileRobot", timeout=5)
        self.request_robot_ops([unload_pxrd_robot_op, robot_wait_op])

    def request_pxrd_process(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("analyse", target_batch=batch)
        self.request_station_op(current_op)
        self.data['batch_analysed'] = True

    ''' transition callbacks'''
    def is_batch_analysed(self):
        return self.data['batch_analysed']


