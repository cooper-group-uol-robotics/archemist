from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import (RobotTaskOp)
from .state import (MTSynthesisStation,
                    MTSynthLoadCartridgeOp,
                    MTSynthUnloadCartridgeOp,
                    MTSynthStopReactionOp,
                    MTSynthHOpenDoorOp,
                    MTSynthHCloseDoorOp)
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any

class CMFlexLiquidDispenseProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            
            State(name='add_liquid', on_enter=['request_adding_liquid']),
            State(name='update_liquid_addition', on_enter=['request_liquid_index_update']),
            
            State(name='open_sash', on_enter=['request_open_sash']),
            State(name='update_open_sash', on_enter=['request_open_sash_update']),
            
            State(name='load_solid_cartridge', on_enter=['request_load_solid_cartridge']),
            State(name='update_loading_cartridge', on_enter=['request_load_cartridge_update']),

            State(name='operate_solid_cartridge', on_enter=['request_operate_solid_cartridge']),
            State(name='add_solid', on_enter=['request_adding_solid']),

            State(name='unload_solid_cartridge', on_enter=['request_unload_solid_cartridge']),
            State(name='update_unloading_cartridge', on_enter=['request_unload_cartridge_update']),

            State(name='close_sash', on_enter=['request_close_sash']),
            State(name='update_close_sash', on_enter=['request_close_sash_update']),

            State(name='sample_reaction', on_enter=['request_sample_reaction']),
            State(name='start_analysis_process', on_enter=['request_analysis_process']),

            State(name='run_reaction', on_enter=['request_reaction_start']),
            
            State(name='wait_for_result'),

            State(name='stop_reaction', on_enter=['request_reaction_stop']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'add_liquid'},
            {'source':'add_liquid','dest':'update_liquid_addition', 'conditions':'are_req_station_ops_completed'},
            {'source':'update_liquid_addition','dest':'add_liquid', 'unless':'are_all_liquids_added'},

            {'source':'update_liquid_addition','dest':'open_sash', 'conditions':'are_all_liquids_added'},
            {'source':'open_sash','dest':'update_open_sash', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_open_sash','dest':'load_solid_cartridge'},


            {'source':'load_solid_cartridge','dest':'update_loading_cartridge', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_loading_cartridge','dest':'operate_solid_cartridge'},
            {'source':'operate_solid_cartridge','dest':'add_solid', 'conditions':'are_req_robot_ops_completed'},
            {'source':'add_solid','dest':'unload_solid_cartridge'},
            {'source':'unload_solid_cartridge','dest':'update_unloading_cartridge', 'conditions':'are_req_robot_ops_completed'},
            
            {'source':'update_unloading_cartridge','dest':'close_sash'},
            {'source':'close_sash','dest':'update_close_sash', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_close_sash','dest':'sample_reaction'},
            
            {'source':'sample_reaction','dest':'start_analysis_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'start_analysis_process','dest':'run_reaction'},
            {'source':'run_reaction','dest':'wait_for_result', 'conditions':'are_req_station_ops_completed'},
            {'source':'wait_for_result','dest':'sample_reaction', 'unless':'is_reaction_complete', 'conditions': 'are_req_station_procs_completed'},
            {'source':'wait_for_result','dest':'stop_reaction', 'conditions':['is_reaction_complete', 'are_req_station_procs_completed']},
            {'source':'stop_reaction','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
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
                                     MTSynthesisStation.__name__,
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["liquids_list"] = liquids_list
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['liquid_index'] = 0

    def request_adding_liquid(self):
        batch = self.lot.batches[0]
        liquid_index = self.data['liquid_index']
        liquid_name = self.data["liquids_list"][liquid_index]

        current_op = self.generate_operation(f"add_{liquid_name}_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_liquid_index_update(self):
        self.data['liquid_index'] += 1

    def open_sash(self):
        robot_task = RobotTaskOp.from_args(name="OpenFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_open_sash_update(self):
        station_op = MTSynthHOpenDoorOp.from_args()
        self.request_station_op(station_op)

    def request_load_solid_cartridge(self):
        params_dict = {}
        params_dict["hotel_index"] = 1
        robot_task = RobotTaskOp.from_args(name="LoadCartridge",
                                           target_robot="KMRIIWARobot",
                                           params=params_dict)
        self.request_robot_ops([robot_task])

    def request_load_cartridge_update(self):
        station_op = MTSynthLoadCartridgeOp.from_args(cartridge_index=1)
        self.request_station_op(station_op)

    def request_operate_solid_cartridge(self):
        robot_task = RobotTaskOp.from_args(name="OperateCartridge",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])
    
    def request_adding_solid(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("add_solid_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_unload_solid_cartridge(self):
        params_dict = {"hotel_index": 1}
        robot_task = RobotTaskOp.from_args(name="UnloadCartridge",
                                           target_robot="KMRIIWARobot",
                                           params=params_dict)
        self.request_robot_ops([robot_task])

    def request_unload_cartridge_update(self):
        station_op = MTSynthUnloadCartridgeOp.from_args()
        self.request_station_op(station_op)

    def close_sash(self):
        robot_task = RobotTaskOp.from_args(name="CloseFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_close_sash_update(self):
        station_op = MTSynthHCloseDoorOp.from_args()
        self.request_station_op(station_op)

    def request_sample_reaction(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("sample_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_analysis_process(self):
        pass
        # from archemist.stations.waters_lcms_station.process import APCAnalysisProcess
        # proc = APCAnalysisProcess.from_args(lot=self.lot)
        # self.request_station_process(proc)

    def request_reaction_start(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("reaction_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    ''' transitions callbacks '''
    def are_all_liquids_added(self):
        liquid_index = self.data['liquid_index']
        num_liquids = len(self.data["liquids_list"])
        return liquid_index == num_liquids

    def is_reaction_complete(self):
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.result_filename == "done"