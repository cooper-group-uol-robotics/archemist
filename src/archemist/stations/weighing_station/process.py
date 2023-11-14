from typing import Dict, Union, List, Any
from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from .state import (
    WeighingStation, 
    WeighingVOpenDoorOp, 
    WeighingVCloseDoorOp, 
    BalanceOpenDoorOp, 
    BalanceCloseDoorOp,
    LoadFunnelOp,
    UnloadFunnelOp,
    TareOp,
    WeighingOp,
    WeighResult
)
from archemist.core.state.robot_op import RobotTaskOp, RobotNavOp, RobotWaitOp
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask, KukaNAVTask
from archemist.core.processing.station_process_fsm import StationProcess 
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location
from archemist.core.state.lot import Lot
import time

class WeighingStationProcess(StationProcess): #TODO StationProcess or StationProcessFSM ?
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [
            State(name='init_state'), 
            State(name='navigate_to_weighing_station', on_enter=['request_navigate_to_weighing']),

            State(name='open_fh_door_vertical', on_enter=['request_open_fh_door_vertical']),
            State(name='update_open_fh_door_vertical', on_enter=['request_open_fh_door_vertical_update']), #TODO need all these update states?

            State(name='tare', on_enter=['request_tare']),

            State(name='open_balance_door', on_enter=['request_open_balance_door']),
            State(name='update_open_balance_door', on_enter=['request_open_balance_door_update']),

            State(name='load_funnel', on_enter=['request_load_funnel']),
            State(name='update_load_funnel', on_enter=['request_load_funnel_update']),

            State(name='close_balance_door', on_enter=['request_close_balance_door']),
            State(name='update_close_balance_door', on_enter=['request_close_balance_door_update']),

            State(name='weigh', on_enter=['request_weigh']),
            #TODO do weigh and tare need an update state? Where WeighOp is called?  Why are these seperated?

            State(name='unload_funnel', on_enter=['request_unload_funnel']),
            State(name='update_unload_funnel', on_enter=['request_unload_funnel_update']),

            State(name='close_fh_door_vertical', on_enter=['request_close_fh_door_vertical']),
            State(name='update_close_fh_door_vertical', on_enter=['request_close_fh_door_vertical_update']),

            State(name='final_state')
        ]
            
        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state','dest':'navigate_to_weighing_station'},
            { 'source':'navigate_to_weighing_station','dest':'open_fh_door_vertical', 'conditions':'are_req_robot_ops_completed'},

            { 'source':'open_fh_door_vertical','dest':'update_open_fh_door_vertical', 'conditions':'are_req_station_ops_completed'},
            { 'source':'update_open_fh_door_vertical','dest':'tare'},

            { 'source':'tare','dest':'open_balance_door', 'conditions':'are_req_station_ops_completed'},

            { 'source':'open_balance_door','dest':'update_open_balance_door', 'conditions':'are_req_station_ops_completed'},
            { 'source':'update_open_balance_door','dest':'load_funnel'},

            { 'source':'load_funnel','dest':'update_load_funnel', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'update_load_funnel','dest':'close_balance_door'},

            { 'source':'close_balance_door','dest':'update_close_balance_door', 'conditions':'are_req_station_ops_completed'},
            { 'source':'update_close_balance_door','dest':'weigh'},

            { 'source':'weigh','dest':'open_balance_door', 'conditions':'are_req_station_ops_completed'}, 

            { 'source':'open_balance_door','dest':'update_open_balance_door', 'conditions':'are_req_station_ops_completed'},
            { 'source':'update_open_balance_door','dest':'unload_funnel'},

            { 'source':'unload_funnel','dest':'update_unload_funnel', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'update_unload_funnel','dest':'close_fh_door_vertical'},

            { 'source':'close_fh_door_vertical','dest':'update_close_fh_door_vertical', 'conditions':'are_req_station_ops_completed'},
            { 'source':'update_close_fh_door_vertical','dest':'final_state'}

        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  liquids_list: List[str],
                  operations: List[Dict[str, Any]] = None,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     WeighingStation.__name__,
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs) #TODO these args correct?
        model.save()
        return cls(model)
    
    ''' States callbacks. '''

    def request_navigate_to_weighing(self):
        # TODO check KUKA command name
        robot_task = RobotNavOp.from_args(name="NavToWeighing",
                                           target_robot="KMRIIWARobot")
        wait_for_next_op = RobotWaitOp.from_args("KMRIIWARobot", 3) #TODO this wait op is correct or no?
        self.request_robot_ops([robot_task, wait_for_next_op])

    def request_open_fh_door_vertical(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("open_v_door_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_open_fh_door_vertical_update(self):
        station_op = WeighingVOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_tare(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("tare_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_open_balance_door(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("open_balance_door_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_open_balance_door_update(self):
        station_op = BalanceOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_funnel(self):
        # TODO check KUKA command name
        robot_task = RobotTaskOp.from_args(name="LoadFunnel",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_load_funnel_update(self):
        station_op = LoadFunnelOp.from_args()
        self.request_station_op(station_op)

    def request_close_balance_door(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("close_balance_door_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_close_balance_door_update(self):
        station_op = BalanceCloseDoorOp.from_args()
        self.request_station_op(station_op)

    def request_weigh(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("weigh_op", target_sample=batch.samples[0]) 
        self.request_station_op(current_op)

    def request_unload_funnel(self):
        # TODO check KUKA command name
        robot_task = RobotTaskOp.from_args(name="UnloadFunnel",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_unload_funnel_update(self):
        station_op = UnloadFunnelOp.from_args()
        self.request_station_op(station_op)

    def request_close_fh_door_vertical(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("close_v_door_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_close_fh_door_vertical_update(self):
        station_op = WeighingVCloseDoorOp.from_args()
        self.request_station_op(station_op)