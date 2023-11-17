from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import (RobotTaskOp)
from .state import (MTSynthesisStation,
                    MTSynthLoadCartridgeOp,
                    MTSynthUnloadCartridgeOp,
                    MTSynthStopReactionOp,
                    MTSynthOpenWindowOp,
                    MTSynthCloseWindowOp,
                    MTSynthOpenReactionValveOp,
                    MTSynthCloseReactionValveOp,
                    MTSynthFilterOp,
                    MTSynthAddWashLiquidOp,
                    MTSynthDryOp,
                    MTSynthReactAndWaitOp,
                    MTSynthReactAndSampleOp,
                    MTSynthDrainOp)
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any
from datetime import datetime, timedelta

class MTAPCSynthProcess(StationProcess):
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
            State(name='open_drain_valve', on_enter=['request_open_drain_valve']),
            State(name='wait_for_drain', on_enter=['start_drain_wait']),
            State(name='close_drain_valve', on_enter=['request_close_drain_valve']),
            State(name='increment_discharge_cycle', on_enter=['request_discharge_cycle_update']),
            State(name='filter_product', on_enter=['request_filtration']),
            State(name='add_wash_liquid', on_enter=['request_adding_wash_liquid']),
            State(name='dry_product', on_enter=['request_dry_product']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'add_liquid'},
            {'source':'add_liquid','dest':'update_liquid_addition', 'conditions':'are_req_station_ops_completed'},
            {'source':'update_liquid_addition','dest':'add_liquid', 'unless':'are_all_liquids_added'},

            {'source':'update_liquid_addition','dest':'open_sash', 'conditions':'are_all_liquids_added'},
            {'source':'open_sash','dest':'update_open_sash', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_open_sash','dest':'load_solid_cartridge', 'conditions':'are_req_station_ops_completed'},


            {'source':'load_solid_cartridge','dest':'update_loading_cartridge', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_loading_cartridge','dest':'operate_solid_cartridge', 'conditions':'are_req_station_ops_completed'},
            {'source':'operate_solid_cartridge','dest':'add_solid', 'conditions':'are_req_robot_ops_completed'},
            {'source':'add_solid','dest':'unload_solid_cartridge', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_solid_cartridge','dest':'update_unloading_cartridge', 'conditions':'are_req_robot_ops_completed'},
            
            {'source':'update_unloading_cartridge','dest':'close_sash', 'conditions':'are_req_station_ops_completed'},
            {'source':'close_sash','dest':'update_close_sash', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_close_sash','dest':'sample_reaction', 'conditions':'are_req_station_ops_completed'},
            
            {'source':'sample_reaction','dest':'start_analysis_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'start_analysis_process','dest':'run_reaction'},
            {'source':'run_reaction','dest':'wait_for_result', 'conditions':'are_req_station_ops_completed'},
            {'source':'wait_for_result','dest':'sample_reaction', 'unless':'is_reaction_complete', 'conditions': 'are_req_station_procs_completed'},
            {'source':'wait_for_result','dest':'stop_reaction', 'conditions':['is_reaction_complete', 'are_req_station_procs_completed']},
            
            {'source':'stop_reaction','dest':'open_drain_valve', 'conditions':'are_req_station_ops_completed'},
            {'source':'open_drain_valve','dest':'wait_for_drain', 'conditions':'are_req_station_ops_completed'},
            {'source':'wait_for_drain','dest':'close_drain_valve', 'conditions':'is_drain_timer_up'},
            {'source':'close_drain_valve','dest':'increment_discharge_cycle', 'conditions':'are_req_station_ops_completed'},
            {'source':'increment_discharge_cycle','dest':'filter_product'},

            {'source':'filter_product','dest':'open_drain_valve',
                "unless": ["is_product_discharged", "is_vessel_clean"],
                'conditions':'are_req_station_ops_completed'},
            
            {'source':'filter_product','dest':'add_wash_liquid',
                "unless": "is_vessel_clean",
                'conditions':['are_req_station_ops_completed', "is_product_discharged"]},
            {'source':'add_wash_liquid','dest':'open_drain_valve', 'conditions':'are_req_station_ops_completed'},

            {'source':'filter_product','dest':'dry_product',
                'conditions':['are_req_station_ops_completed', "is_product_discharged", "is_vessel_clean"]},
            {'source':'dry_product','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  num_liquids: int,
                  min_concentration: float,
                  drain_duration: int,
                  cycles_to_discharge: int,
                  cycles_to_clean: int,
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
        model.data["num_liquids"] = num_liquids
        model.data["min_concentration"] = min_concentration
        model.data["drain_duration"] = drain_duration
        model.data["cycles_to_discharge"] = cycles_to_discharge
        model.data["max_num_cycles"] = cycles_to_clean + cycles_to_discharge
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['liquid_index'] = 0
        self.data['num_discharge_cycles'] = 0

    def request_adding_liquid(self):
        batch = self.lot.batches[0]
        liquid_index = self.data['liquid_index']

        current_op = self.generate_operation(f"add_liquid_op_{liquid_index+1}", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_liquid_index_update(self):
        self.data['liquid_index'] += 1

    def request_open_sash(self):
        robot_task = RobotTaskOp.from_args(name="OpenFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_open_sash_update(self):
        station_op = MTSynthOpenWindowOp.from_args()
        self.request_station_op(station_op)

    def request_load_solid_cartridge(self):
        robot_task = RobotTaskOp.from_args(name="LoadCartridge",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_load_cartridge_update(self):
        station_op = MTSynthLoadCartridgeOp.from_args(cartridge_index=0)
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
        robot_task = RobotTaskOp.from_args(name="UnloadCartridge",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_unload_cartridge_update(self):
        station_op = MTSynthUnloadCartridgeOp.from_args()
        self.request_station_op(station_op)

    def request_close_sash(self):
        robot_task = RobotTaskOp.from_args(name="CloseFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_close_sash_update(self):
        station_op = MTSynthCloseWindowOp.from_args()
        self.request_station_op(station_op)

    def request_sample_reaction(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("sample_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_analysis_process(self):
        from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot)
        self.request_station_process(proc)

    def request_reaction_start(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("reaction_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    def request_open_drain_valve(self):
        station_op = MTSynthOpenReactionValveOp.from_args()
        self.request_station_op(station_op)

    def start_drain_wait(self):
        drain_duration = self.data["drain_duration"]
        drain_finish_time = datetime.now() + timedelta(seconds=drain_duration)
        self.data["drain_finish_time"] = drain_finish_time.isoformat()
    
    def request_close_drain_valve(self):
        station_op = MTSynthCloseReactionValveOp.from_args()
        self.request_station_op(station_op)

    def request_filtration(self):
        station_op = MTSynthFilterOp.from_args()
        self.request_station_op(station_op)

    def request_adding_wash_liquid(self):
        station_op = MTSynthAddWashLiquidOp.from_args(liquid_name="H2O",
                                                      dispense_volume=10,
                                                      dispense_unit="mL")
        self.request_station_op(station_op)

    def request_dry_product(self):
        station_op = MTSynthDryOp.from_args(duration=5,
                                            time_unit="minute")
        self.request_station_op(station_op)

    def request_discharge_cycle_update(self):
        self.data["num_discharge_cycles"] += 1

    ''' transitions callbacks '''
    def are_all_liquids_added(self):
        liquid_index = self.data['liquid_index']
        num_liquids = self.data["num_liquids"]
        return liquid_index == num_liquids

    def is_reaction_complete(self):
        min_concentration = self.data["min_concentration"]
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration >= min_concentration

    def is_drain_timer_up(self):
        current_time = datetime.now()
        finish_time = datetime.fromisoformat(self.data["drain_finish_time"])
        return current_time >= finish_time

    def is_product_discharged(self):
        return self.data["num_discharge_cycles"] >= self.data["cycles_to_discharge"]

    def is_vessel_clean(self):
        return self.data["num_discharge_cycles"] == self.data["max_num_cycles"]

class MTAPCFilterProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 

            State(name='add_wash_liquid', on_enter=['request_adding_wash_liquid']),

            State(name='run_reaction', on_enter=['request_reaction_start']),
            State(name='sample_reaction', on_enter=['request_sample_reaction']),
            State(name='start_analysis_process', on_enter=['request_analysis_process']),
            State(name='stop_reaction', on_enter=['request_reaction_stop']),

            State(name='open_drain_valve', on_enter=['request_open_drain_valve']),
            State(name='wait_for_drain', on_enter=['start_drain_wait']),
            State(name='close_drain_valve', on_enter=['request_close_drain_valve']),
            
            State(name='drain_filter', on_enter=['request_filter_drain']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'add_wash_liquid'},
            {'source':'add_wash_liquid','dest':'run_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'run_reaction','dest':'stop_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'stop_reaction','dest':'sample_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'sample_reaction','dest':'start_analysis_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'start_analysis_process','dest':'open_drain_valve', 'conditions':'are_req_station_procs_completed'},            
            {'source':'open_drain_valve','dest':'wait_for_drain', 'conditions':'are_req_station_ops_completed'},
            {'source':'wait_for_drain','dest':'close_drain_valve', 'conditions':'is_drain_timer_up'},
            {'source':'close_drain_valve','dest':'drain_filter', 'conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'add_wash_liquid', 'unless': 'is_reactor_clean','conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'final_state', 'conditions':['are_req_station_ops_completed', 'is_reactor_clean']},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  max_concentration: float,
                  drain_duration: int,
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
        model.data["max_concentration"] = max_concentration
        model.data["drain_duration"] = drain_duration
        model.save()
        return cls(model)

    ''' state callbacks '''
    def request_adding_wash_liquid(self):
        station_op = MTSynthAddWashLiquidOp.from_args(liquid_name="H2O",
                                                      dispense_volume=10,
                                                      dispense_unit="mL")
        self.request_station_op(station_op)
    
    def request_reaction_start(self):
        batch = self.lot.batches[0]
        current_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
                                                     target_temperature=100,
                                                     target_stirring_speed=50,
                                                     wait_duration=5,
                                                     time_unit="minute")
        self.request_station_op(current_op)

    def request_analysis_process(self):
        from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot)
        self.request_station_process(proc)

    def request_sample_reaction(self):
        batch = self.lot.batches[0]
        current_op = MTSynthReactAndSampleOp.from_args(target_sample=batch.samples[0],
                                                     target_temperature=100,
                                                     target_stirring_speed=50)
        self.request_station_op(current_op)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    def request_open_drain_valve(self):
        station_op = MTSynthOpenReactionValveOp.from_args()
        self.request_station_op(station_op)

    def start_drain_wait(self):
        drain_duration = self.data["drain_duration"]
        drain_finish_time = datetime.now() + timedelta(seconds=drain_duration)
        self.data["drain_finish_time"] = drain_finish_time.isoformat()
    
    def request_close_drain_valve(self):
        station_op = MTSynthCloseReactionValveOp.from_args()
        self.request_station_op(station_op)

    def request_filter_drain(self):
        station_op = MTSynthDrainOp.from_args()
        self.request_station_op(station_op)

    ''' transitions callbacks '''
    def is_reactor_clean(self):
        max_concentration = self.data["max_concentration"]
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration <= max_concentration

    def is_drain_timer_up(self):
        current_time = datetime.now()
        finish_time = datetime.fromisoformat(self.data["drain_finish_time"])
        return current_time >= finish_time