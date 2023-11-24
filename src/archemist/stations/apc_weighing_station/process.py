from typing import Dict, Union, List, Any
from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from .state import (
    APCWeighingOpenVDoorOp, 
    APCWeighingCloseVDoorOp, 
    APCOpenBalanceDoorOp, 
    APCCloseBalanceDoorOp,
    APCTareOp,
    APCWeighingOp,
    APCWeighingStation
)
from archemist.core.state.robot_op import RobotTaskOp, RobotNavOp, RobotWaitOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.lot import Lot

class APCWeighingProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [
            State(name='init_state'), 
            State(name='prep_state', on_enter=['initialise_process_data']),
            State(name='navigate_to_weighing_station', on_enter=['request_navigate_to_weighing']),
            State(name='open_fh_door_vertical', on_enter=['request_open_fh_door_vertical']),
            State(name='tare', on_enter=['request_tare']),
            State(name='open_balance_door', on_enter=['request_open_balance_door']),
            State(name='load_funnel', on_enter=['request_load_funnel']),
            State(name='close_balance_door', on_enter=['request_close_balance_door']),
            State(name='weigh', on_enter=['request_weigh']),
            State(name='unload_funnel', on_enter=['request_unload_funnel']),
            State(name='close_fh_door_vertical', on_enter=['request_close_fh_door_vertical']),
            State(name='final_state')
        ]
            
        ''' Transitions '''
        self.TRANSITIONS = [
            { 'source':'init_state','dest':'prep_state'},
            { 'source':'prep_state','dest':'navigate_to_weighing_station'},
            { 'source':'navigate_to_weighing_station','dest':'open_fh_door_vertical', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'open_fh_door_vertical','dest':'tare', 'conditions':'are_req_station_ops_completed'},
            { 'source':'tare','dest':'open_balance_door', 'conditions':'are_req_station_ops_completed'},
            { 'source':'open_balance_door','dest':'load_funnel', 'unless':'is_weighing_complete', 'conditions':'are_req_station_ops_completed'},
            { 'source':'load_funnel','dest':'close_balance_door', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'close_balance_door','dest':'weigh', 'conditions':'are_req_station_ops_completed'},
            { 'source':'weigh','dest':'open_balance_door', 'conditions':'are_req_station_ops_completed'}, 
            { 'source':'open_balance_door','dest':'unload_funnel', 'conditions':['are_req_station_ops_completed','is_weighing_complete']},
            { 'source':'unload_funnel','dest':'close_fh_door_vertical', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'close_fh_door_vertical','dest':'final_state', 'conditions':'are_req_station_ops_completed'}
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     APCWeighingStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)
    
    ''' States callbacks. '''

    def initialise_process_data(self):
        self.data['is_weighing_complete'] = False

    def request_navigate_to_weighing(self):
        robot_task = RobotNavOp.from_args(
            name="NavToWeighing",
            target_robot="KMRIIWARobot",
            target_location=None
        )
        wait_for_next_op = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_for_next_op])

    def request_open_fh_door_vertical(self):
        station_op = APCWeighingOpenVDoorOp.from_args()
        self.request_station_op(station_op)

    def request_tare(self):
        station_op = APCTareOp.from_args()
        self.request_station_op(station_op)

    def request_open_balance_door(self):
        station_op = APCOpenBalanceDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_funnel(self):
        robot_task = RobotTaskOp.from_args(
            name="LoadFunnel",
            target_robot="KMRIIWARobot"
        )
        wait_for_next_op = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_for_next_op])

    def request_close_balance_door(self):
        station_op = APCCloseBalanceDoorOp.from_args()
        self.request_station_op(station_op)

    def request_weigh(self):
        batch = self.lot.batches[0]
        station_op = APCWeighingOp.from_args(target_sample=batch.samples[0])
        self.request_station_op(station_op)
        self.data['is_weighing_complete'] = True

    def request_unload_funnel(self):
        # TODO add parameter for funnel index?
        robot_task = RobotTaskOp.from_args(name="UnloadFunnel",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_close_fh_door_vertical(self):
        station_op = APCWeighingCloseVDoorOp.from_args()
        self.request_station_op(station_op)

    ''' Transition callbacks. '''

    def is_weighing_complete(self):
        return self.data['is_weighing_complete']