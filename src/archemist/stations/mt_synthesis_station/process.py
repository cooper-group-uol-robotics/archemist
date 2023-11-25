from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import (RobotTaskOp)
from .state import (MTSynthesisStation,
                    MTSynthStopReactionOp,
                    MTSynthTimedOpenReactionValveOp,
                    MTSynthFilterOp,
                    MTSynthAddWashLiquidOp,
                    MTSynthReactAndWaitOp,
                    MTSynthReactAndSampleOp,
                    MTSynthDrainOp)
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any

class MTAPCSynthProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            
            State(name='add_liquid', on_enter=['request_adding_liquid']),
            State(name='update_liquid_addition', on_enter=['request_liquid_index_update']),
            
            State(name='open_sash', on_enter=['request_open_sash']),
            State(name='update_open_sash', on_enter=['set_sash_to_open']),
            
            State(name='load_solid_cartridge', on_enter=['request_load_solid_cartridge']),
            State(name='update_loading_cartridge', on_enter=['set_station_loaded_cartridge']),

            State(name='operate_solid_cartridge', on_enter=['request_operate_solid_cartridge']),
            State(name='add_solid', on_enter=['request_adding_solid']),

            State(name='unload_solid_cartridge', on_enter=['request_unload_solid_cartridge']),
            State(name='update_unloading_cartridge', on_enter=['unset_station_loaded_cartridge']),

            State(name='close_sash', on_enter=['request_close_sash']),
            State(name='update_close_sash', on_enter=['set_sash_to_closed']),

            State(name='sample_reaction', on_enter=['request_sample_reaction']),
            State(name='start_analysis_process', on_enter=['request_analysis_process']),

            State(name='run_reaction', on_enter=['request_reaction_start']),
            
            State(name='wait_for_result'),

            State(name='stop_reaction', on_enter=['request_reaction_stop']),
            State(name='filteration_process', on_enter=['request_filteration_process']),
            State(name='queue_cleaning_process', on_enter='request_cleaning_process'),
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
            
            {'source':'stop_reaction','dest':'filteration_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'filteration_process','dest':'queue_cleaning_process', 'conditions':'are_req_station_procs_completed'},
            {'source':'queue_cleaning_process','dest':'final_state'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_product_concentration: float,
                  target_purity_concentration: float,
                  num_discharge_cycles: int,
                  num_wash_cycles: int,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     MTSynthesisStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_product_concentration"] = float(target_product_concentration)
        model.data["target_purity_concentration"] = float(target_purity_concentration)
        model.data["num_discharge_cycles"] = num_discharge_cycles
        model.data["num_wash_cycles"] = num_wash_cycles
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['liquid_index'] = 0
        num_liquids = 0
        for op_names, _ in self.operation_specs_map.items():
            num_liquids += 1 if "add_liquid" in op_names else 0
        self.data['num_liquids'] = num_liquids

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

    def set_sash_to_open(self):
        synth_station: MTSynthesisStation = self.get_assigned_station()
        synth_station.window_open = True

    def request_load_solid_cartridge(self):
        synth_station: MTSynthesisStation = self.get_assigned_station()
        params_dict = {}
        for cartridge in synth_station.cartridges:
            if not cartridge.depleted:
                params_dict["hotel_index"] = cartridge.hotel_index
                break
        robot_task = RobotTaskOp.from_args(name="LoadCartridge",
                                           target_robot="KMRIIWARobot",
                                           params=params_dict)
        self.request_robot_ops([robot_task])

    def set_station_loaded_cartridge(self):
        synth_station: MTSynthesisStation = self.get_assigned_station()
        for index, cartridge in enumerate(synth_station.cartridges):
            if not cartridge.depleted:
                synth_station.load_cartridge(index)
                break

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

    def unset_station_loaded_cartridge(self):
        synth_station: MTSynthesisStation = self.get_assigned_station()
        synth_station.unload_cartridge()

    def request_close_sash(self):
        robot_task = RobotTaskOp.from_args(name="CloseFumeHoodSash",
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def set_sash_to_closed(self):
        synth_station: MTSynthesisStation = self.get_assigned_station()
        synth_station.window_open = False

    def request_sample_reaction(self):
        batch = self.lot.batches[0]
        reaction_operation = self.operation_specs_map["reaction_op"]
        reaction_temperature = reaction_operation.parameters["target_temperature"]
        reaction_stirring_speed = reaction_operation.parameters["target_stirring_speed"]
        op = MTSynthReactAndSampleOp.from_args(target_sample=batch.samples[0],
                                               target_temperature=reaction_temperature,
                                               target_stirring_speed=reaction_stirring_speed)
        self.request_station_op(op)

    def request_analysis_process(self):
        from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot, is_subprocess=True)
        self.request_station_process(proc)

    def request_reaction_start(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("reaction_op", target_sample=batch.samples[0])
        self.request_station_op(current_op)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    def request_filteration_process(self):
        discharge_operation = self.operation_specs_map["discharge_product"]
        wash_operation = self.operation_specs_map["wash_product"]
        dry_operation = self.operation_specs_map["dry_product"]
        operations = [{
            "name": "discharge_product",
            "op": discharge_operation.op_type,
            "parameters": dict(discharge_operation.parameters)
        },
        {
            "name": "wash_product",
            "op": wash_operation.op_type,
            "parameters": dict(wash_operation.parameters)
        },
        {
            "name": "dry_product",
            "op": dry_operation.op_type,
            "parameters": dict(dry_operation.parameters)
        }]
        num_discharge_cycles = self.data["num_discharge_cycles"]
        num_wash_cycles = self.data["num_wash_cycles"]
        proc = MTAPCFilterProcess.from_args(lot=self.lot,
                                           operations=operations,
                                           num_discharge_cycles=num_discharge_cycles,
                                           num_wash_cycles=num_wash_cycles,
                                           is_subprocess=True)
        self.request_station_process(proc)

    def request_cleaning_process(self):
        purity_concentration = self.data["target_purity_concentration"]
        proc = MTAPCCleanProcess.from_args(lot=self.lot,
                                           target_purity_concentration=purity_concentration,
                                           is_subprocess=True)
        self.request_station_process(proc)

    ''' transitions callbacks '''
    def are_all_liquids_added(self):
        liquid_index = self.data['liquid_index']
        num_liquids = self.data["num_liquids"]
        return liquid_index == num_liquids

    def is_reaction_complete(self):
        target_product_concentration = self.data["target_product_concentration"]
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration >= target_product_concentration
    
class MTAPCFilterProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='open_close_drain_valve', on_enter=['request_open_close_drain_valve']),
            State(name='increment_discharge_cycle', on_enter=['request_discharge_cycle_update']),
            State(name='filter_product', on_enter=['request_filtration']),
            State(name='add_wash_liquid', on_enter=['request_adding_wash_liquid']),
            State(name='dry_product', on_enter=['request_dry_product']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'open_close_drain_valve'},
            
            {'source':'open_close_drain_valve','dest':'increment_discharge_cycle', 'conditions':'are_req_station_ops_completed'},
            {'source':'increment_discharge_cycle','dest':'filter_product'},

            {'source':'filter_product','dest':'open_close_drain_valve',
                "unless": ["is_product_discharged", "is_vessel_clean"],
                'conditions':'are_req_station_ops_completed'},
            
            {'source':'filter_product','dest':'add_wash_liquid',
                "unless": "is_vessel_clean",
                'conditions':['are_req_station_ops_completed', "is_product_discharged"]},
            {'source':'add_wash_liquid','dest':'open_close_drain_valve', 'conditions':'are_req_station_ops_completed'},

            {'source':'filter_product','dest':'dry_product',
                'conditions':['are_req_station_ops_completed', "is_product_discharged", "is_vessel_clean"]},
            {'source':'dry_product','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  num_discharge_cycles: int,
                  num_wash_cycles: int,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     MTSynthesisStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["max_num_discharge_cycles"] = int(num_discharge_cycles)
        model.data["max_num_cycles"] = int(num_wash_cycles) + int(num_discharge_cycles)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['num_discharge_cycles'] = 0

    def request_open_close_drain_valve(self):
        station_op = self.generate_operation("discharge_product")
        self.request_station_op(station_op)

    def request_filtration(self):
        station_op = MTSynthFilterOp.from_args()
        self.request_station_op(station_op)

    def request_adding_wash_liquid(self):
        station_op = self.generate_operation("wash_product")
        self.request_station_op(station_op)

    def request_dry_product(self):
        station_op = self.generate_operation("dry_product")
        self.request_station_op(station_op)

    def request_discharge_cycle_update(self):
        self.data["num_discharge_cycles"] += 1

    ''' transitions callbacks '''
    def is_product_discharged(self):
        return self.data["num_discharge_cycles"] >= self.data["max_num_discharge_cycles"]

    def is_vessel_clean(self):
        return self.data["num_discharge_cycles"] == self.data["max_num_cycles"]

class MTAPCCleanProcess(StationProcess):
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

            State(name='open_close_drain_valve', on_enter=['request_open_close_drain_valve']),
            
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
            {'source':'start_analysis_process','dest':'open_close_drain_valve', 'conditions':'are_req_station_procs_completed'},            
            {'source':'open_close_drain_valve','dest':'drain_filter', 'conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'add_wash_liquid', 'unless': 'is_reactor_clean','conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'final_state', 'conditions':['are_req_station_ops_completed', 'is_reactor_clean']},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_purity_concentration: float,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     MTSynthesisStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_purity_concentration"] = float(target_purity_concentration)
        model.save()
        return cls(model)

    ''' state callbacks '''
    def request_adding_wash_liquid(self):
        station_op = MTSynthAddWashLiquidOp.from_args(liquid_name="water",
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
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot, is_subprocess=True)
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

    def request_open_close_drain_valve(self):
        station_op = MTSynthTimedOpenReactionValveOp.from_args(duration=2,
                                                               time_unit="second")
        self.request_station_op(station_op)

    def request_filter_drain(self):
        station_op = MTSynthDrainOp.from_args()
        self.request_station_op(station_op)

    ''' transitions callbacks '''
    def is_reactor_clean(self):
        target_purity_concentration = self.data["target_purity_concentration"]
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration <= target_purity_concentration