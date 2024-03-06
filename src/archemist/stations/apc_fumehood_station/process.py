from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp
from .state import APCFumehoodStation
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any


class APCSolidAdditionProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'),
                       State(name='prep_state'),

                       State(name='open_slide_window', on_enter=['request_open_slide_window']),
                       State(name='update_open_slide_window', on_enter=['set_slide_window_to_open']),

                       State(name='load_solid_cartridge', on_enter=['request_load_solid_cartridge']),
                       State(name='update_loading_cartridge', on_enter=['set_station_loaded_cartridge']),

                       State(name='operate_solid_cartridge', on_enter=['request_operate_solid_cartridge']),
                       State(name='add_solid', on_enter=['request_adding_solid']),

                       State(name='unload_solid_cartridge', on_enter=['request_unload_solid_cartridge']),
                       State(name='update_unloading_cartridge', on_enter=['unset_station_loaded_cartridge']),

                       State(name='close_slide_window', on_enter=['request_close_slide_window']),
                       State(name='update_close_slide_window', on_enter=['set_slide_window_to_closed']),

                       State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source': 'init_state', 'dest': 'prep_state'},
            {'source': 'prep_state', 'dest': 'open_slide_window'},

            {'source': 'open_slide_window', 'dest': 'update_open_slide_window', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'update_open_slide_window', 'dest': 'load_solid_cartridge'},


            {'source': 'load_solid_cartridge', 'dest': 'update_loading_cartridge', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'update_loading_cartridge', 'dest': 'operate_solid_cartridge'},
            {'source': 'operate_solid_cartridge', 'dest': 'add_solid', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'add_solid', 'dest': 'unload_solid_cartridge', 'conditions': 'are_req_station_ops_completed'},
            {'source': 'unload_solid_cartridge', 'dest': 'update_unloading_cartridge', 'conditions': 'are_req_robot_ops_completed'},

            {'source': 'update_unloading_cartridge', 'dest': 'close_slide_window'},
            {'source': 'close_slide_window', 'dest': 'update_close_slide_window', 'conditions': 'are_req_robot_ops_completed'},
            {'source': 'update_close_slide_window', 'dest': 'final_state'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_batch_index: int,
                  target_sample_index: int,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool = False,
                  skip_robot_ops: bool = False,
                  skip_station_ops: bool = False,
                  skip_ext_procs: bool = False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     APCFumehoodStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_batch_index"] = target_batch_index
        model.data["target_sample_index"] = target_sample_index
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_open_slide_window(self):
        robot_task = RobotTaskOp.from_args(name="OpenFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_task])

    def set_slide_window_to_open(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        synth_station.slide_window_open = True

    def request_load_solid_cartridge(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        params_dict = {}
        for cartridge in synth_station.cartridges:
            if not cartridge.depleted:
                params_dict["hotel_index"] = cartridge.hotel_index
                break
        robot_task = RobotTaskOp.from_args(name="LoadCartridge",
                                           target_robot="KMRIIWARobot",
                                           params=params_dict)
        wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_task])

    def set_station_loaded_cartridge(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        for index, cartridge in enumerate(synth_station.cartridges):
            if not cartridge.depleted:
                synth_station.load_cartridge(index)
                break

    def request_operate_solid_cartridge(self):
        robot_task = RobotTaskOp.from_args(name="OperateCartridge",
                                           target_robot="KMRIIWARobot")
        wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_task])

    def request_adding_solid(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("add_solid", target_sample=sample)
        self.request_station_op(current_op)

    def request_unload_solid_cartridge(self):
        robot_task = RobotTaskOp.from_args(name="UnloadCartridge",
                                           target_robot="KMRIIWARobot")
        wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_task])

    def unset_station_loaded_cartridge(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        synth_station.unload_cartridge()

    def request_close_slide_window(self):
        robot_task = RobotTaskOp.from_args(name="CloseFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def set_slide_window_to_closed(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        synth_station.slide_window_open = False
