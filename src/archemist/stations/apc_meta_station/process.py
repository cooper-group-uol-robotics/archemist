from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp, RobotNavOp

from archemist.stations.apc_fumehood_station.process import APCSolidAdditionProcess
from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
from archemist.stations.apc_weighing_station.process import APCWeighingProcess, APCNewFunnelProcess

from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
from archemist.stations.syringe_pump_station.state import SyringePumpFinishDispensingOp
from archemist.stations.mt_synthesis_station.state import MTSynthSampleOp, MTSynthStopReactionOp, MTSynthCustomOpenCloseReactionValveOp, MTSynthLongOpenCloseReactionValveOp
from archemist.stations.apc_filtration_station.state import APCFilterProductOp, APCDrainWasteOp
from archemist.stations.apc_fumehood_station.state import APCOpenSashOp, APCCloseSashOp
from archemist.stations.apc_weighing_station.state import APCWeighingStation

from archemist.core.state.lot import Lot
from .state import APCMetaStation
import time

from typing import Union, List, Dict, Any

class APCSynthesisProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter=['initialise_process_data']), 
            State(name='load_funnel', on_enter=['request_load_funnel_process']), 
            
            State(name='add_liquid_1', on_enter=['request_adding_liquid_1']),
            
            State(name='add_solid', on_enter=['request_solid_addition_process']),

            State(name='sample_reaction', on_enter=['request_sample_reaction']),
            State(name='start_analysis_process', on_enter=['request_analysis_process']),
            State(name='add_liquid_2', on_enter=['request_adding_liquid_2']),

            State(name='run_reaction', on_enter=['request_reaction_start']),
            
            State(name='wait_for_result'), # TODO nothing here! Need to check for LCMS results

            State(name='stop_reaction', on_enter=['request_reaction_stop']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'load_funnel'},
            {'source':'load_funnel', 'dest': 'add_liquid_1', 'conditions':'are_req_station_procs_completed'},
            {'source':'add_liquid_1','dest':'add_solid', 'conditions':'are_req_station_ops_completed'},
            {'source':'add_solid','dest':'sample_reaction', 'conditions':'are_req_station_procs_completed'},
            
            {'source':'sample_reaction','dest':'start_analysis_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'start_analysis_process','dest':'run_reaction', 'conditions':'are_req_station_procs_completed'},

            {'source':'run_reaction','dest':'add_liquid_2', 'unless':'is_liquid_2_added', 'conditions':'are_req_station_ops_completed'},
            {'source':'run_reaction','dest':'wait_for_result', 'conditions':'are_req_station_ops_completed'},

            {'source':'add_liquid_2','dest':'wait_for_result', 'conditions':'are_req_station_ops_completed'},
            
            {'source':'wait_for_result','dest':'sample_reaction', 'unless':'is_reaction_complete', 'conditions': 'are_req_station_procs_completed'},
            {'source':'wait_for_result','dest':'stop_reaction', 'conditions':['is_reaction_complete', 'are_req_station_procs_completed']},
            
            {'source':'stop_reaction','dest':'final_state', 'conditions':'are_req_station_ops_completed'}
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_product_concentration: float,
                  target_batch_index: int,
                  target_sample_index: int,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     APCMetaStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_product_concentration"] = float(target_product_concentration)
        model.data["target_batch_index"] = int(target_batch_index)
        model.data["target_sample_index"] = int(target_sample_index)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['is_liquid_2_added'] = False

    def request_load_funnel_process(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        add_solid_operation = self.operation_specs_map["add_solid"]
        operations = [{
            "name": "add_solid",
            "op": add_solid_operation.op_type,
            "parameters": dict(add_solid_operation.parameters)
        }]

        proc = APCNewFunnelProcess.from_args(lot=self.lot,
                                                 target_batch_index=batch_index,
                                                 target_sample_index=sample_index,
                                                 operations=operations)
        self.request_station_process(proc)

    def request_adding_liquid_1(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation(f"add_liquid_1", target_sample=sample)
        self.request_station_op(current_op)

    def request_solid_addition_process(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        add_solid_operation = self.operation_specs_map["add_solid"]
        operations = [{
            "name": "add_solid",
            "op": add_solid_operation.op_type,
            "parameters": dict(add_solid_operation.parameters)
        }]

        proc = APCSolidAdditionProcess.from_args(lot=self.lot,
                                                 target_batch_index=batch_index,
                                                 target_sample_index=sample_index,
                                                 operations=operations)
        self.request_station_process(proc)

    def request_adding_liquid_2(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation(f"add_liquid_2", target_sample=sample)
        self.request_station_op(current_op)
        self.data['is_liquid_2_added'] = True

    def request_sample_reaction(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        reaction_operation = self.operation_specs_map["heat_stir"]
        reaction_temperature = reaction_operation.parameters["target_temperature"]
        reaction_stirring_speed = reaction_operation.parameters["target_stirring_speed"]
        op = MTSynthSampleOp.from_args(target_sample=sample,
                                        target_temperature=reaction_temperature,
                                        target_stirring_speed=reaction_stirring_speed)
        self.request_station_op(op)

    def request_analysis_process(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot,
                                                target_batch_index=batch_index,
                                                target_sample_index=sample_index)
        self.request_station_process(proc)

    def request_reaction_start(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("heat_stir", target_sample=sample)
        self.request_station_op(current_op)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    ''' transitions callbacks '''

    def is_liquid_2_added(self):
        return self.data['is_liquid_2_added']

    def is_reaction_complete(self):
        target_product_concentration = self.data["target_product_concentration"]
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration >= target_product_concentration

class APCFiltrationProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='start_heat_stir', on_enter=['request_heating_stirring']), 
            State(name='wait_for_crystallisation', on_enter=['request_wait_for_crystallisation']), 
            State(name='open_close_drain_valve_custom', on_enter=['request_open_close_drain_valve_custom']),
            State(name='open_close_drain_valve_long', on_enter=['request_open_close_drain_valve_long']),
            State(name='increment_discharge_cycle', on_enter=['request_discharge_cycle_update']),
            State(name='filter_product', on_enter=['request_filtration']),
            State(name='add_wash_liquid', on_enter=['request_adding_wash_liquid']),
            State(name='stop_heat_stir', on_enter=['request_stop_heating_stirring']),
            State(name='dry_product', on_enter=['request_dry_product']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'start_heat_stir'},
            {'source':'start_heat_stir', 'dest': 'wait_for_crystallisation', 'conditions':'are_req_station_ops_completed'},
            {'source':'wait_for_crystallisation', 'dest': 'open_close_drain_valve_custom', 'conditions':'are_req_station_ops_completed'},
            
            {'source':'open_close_drain_valve_custom','dest':'increment_discharge_cycle', 'conditions':'are_req_station_ops_completed'},
            {'source':'increment_discharge_cycle','dest':'filter_product'},

            {'source':'filter_product','dest':'open_close_drain_valve_custom',
                "unless": "is_last_discharge",
                'conditions':'are_req_station_ops_completed'},
            {'source':'filter_product','dest':'open_close_drain_valve_long',
                'conditions':['are_req_station_ops_completed', "is_last_discharge"]},
            
            {'source':'open_close_drain_valve_long','dest':'add_wash_liquid', "unless":"is_vessel_clean",'conditions':'are_req_station_ops_completed'},
            {'source':'open_close_drain_valve_long','dest':'stop_heat_stir', 'conditions':['are_req_station_ops_completed', "is_vessel_clean"]},

            {'source':'add_wash_liquid','dest':'increment_discharge_cycle', 'conditions':'are_req_station_ops_completed'},

            {'source':'stop_heat_stir','dest':'dry_product', 'conditions':'are_req_station_ops_completed'},
            {'source':'dry_product','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_batch_index: int,
                  target_sample_index: int,
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
                                     APCMetaStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["max_num_discharge_cycles"] = int(num_discharge_cycles)
        model.data["max_num_cycles"] = int(num_wash_cycles)*int(num_discharge_cycles)
        model.data["target_batch_index"] = int(target_batch_index)
        model.data["target_sample_index"] = int(target_sample_index)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['num_discharge_cycles'] = 0

    def request_heating_stirring(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("heat_stir_discharge", target_sample=sample)
        self.request_station_op(current_op)

    def request_wait_for_crystallisation(self):
        time.sleep(1800) # TODO how should this be done properly?

    def request_open_close_drain_valve_long(self):
        station_op = MTSynthLongOpenCloseReactionValveOp.from_args()
        self.request_station_op(station_op)

    def request_open_close_drain_valve_custom(self):
        station_op = self.generate_operation("custom_discharge")
        self.request_station_op(station_op)

    def request_filtration(self):
        station_op = APCFilterProductOp.from_args()
        self.request_station_op(station_op)

    def request_adding_wash_liquid(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]

        station_op = self.generate_operation("wash_product", target_sample=sample)
        self.request_station_op(station_op)

    def request_stop_heating_stirring(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    def request_dry_product(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        station_op = self.generate_operation("dry_product", target_sample=sample)
        self.request_station_op(station_op)

    def request_discharge_cycle_update(self):
        self.data["num_discharge_cycles"] += 1

    ''' transitions callbacks '''
    def is_last_discharge(self):
        if (self.data["num_discharge_cycles"] % self.data["max_num_discharge_cycles"] == 0) and (self.data["num_discharge_cycles"] != 0):
            return True
        else:
            return False

    def is_vessel_clean(self):
        return self.data["num_discharge_cycles"] == self.data["max_num_cycles"]

class APCCleaningProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 

            State(name='load_cleaning_funnel', on_enter=['request_load_cleaning_funnel']),
            State(name='add_wash_liquid', on_enter=['request_adding_wash_liquid']),

            State(name='run_reaction', on_enter=['request_reaction_start']),
            State(name='sample_reaction', on_enter=['request_sample_reaction']),
            State(name='start_analysis_process', on_enter=['request_analysis_process']),
            State(name='stop_reaction', on_enter=['request_reaction_stop']),

            State(name='open_close_drain_valve', on_enter=['request_open_close_drain_valve']),
            State(name='unload_cleaning_funnel', on_enter=['request_unload_cleaning_funnel']),
            State(name='drain_filter', on_enter=['request_filter_drain']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'load_cleaning_funnel'},
            {'source':'load_cleaning_funnel', 'dest': 'add_wash_liquid', 'conditions': 'are_req_robot_ops_completed'},
            {'source':'add_wash_liquid','dest':'run_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'run_reaction','dest':'stop_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'stop_reaction','dest':'sample_reaction', 'conditions':'are_req_station_ops_completed'},
            {'source':'sample_reaction','dest':'start_analysis_process', 'conditions':'are_req_station_ops_completed'},
            {'source':'start_analysis_process','dest':'open_close_drain_valve', 'conditions':'are_req_station_procs_completed'},            
            {'source':'open_close_drain_valve','dest':'drain_filter', 'conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'add_wash_liquid', 'unless': 'is_reactor_clean','conditions':'are_req_station_ops_completed'},
            {'source':'drain_filter','dest':'unload_cleaning_funnel', 'conditions':['are_req_station_ops_completed', 'is_reactor_clean']},
            {'source':'unload_cleaning_funnel', 'dest': 'final_state', 'conditions': 'are_req_robot_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_batch_index: int,
                  target_sample_index: int,
                  target_purity_concentration: float,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     APCMetaStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_purity_concentration"] = float(target_purity_concentration)
        model.data["target_batch_index"] = int(target_batch_index)
        model.data["target_sample_index"] = int(target_sample_index)
        model.save()
        return cls(model)

    ''' state callbacks '''
    def request_load_cleaning_funnel(self):
        robot_task = RobotTaskOp.from_args(
            name="loadEmptyFunnel",
            target_robot="KMRIIWARobot",
            task_type = 2,
            lbr_program_name = "loadEmptyFunnel"
        )
        self.request_robot_ops([robot_task])
        
    def request_adding_wash_liquid(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        station_op = self.generate_operation("add_wash_liquid", target_sample=sample)
        self.request_station_op(station_op)
    
    def request_reaction_start(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("wash_heat_stir", target_sample=sample)
        self.request_station_op(current_op)

    def request_sample_reaction(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        reaction_operation = self.operation_specs_map["wash_heat_stir"]
        reaction_temperature = reaction_operation.parameters["target_temperature"]
        reaction_stirring_speed = reaction_operation.parameters["target_stirring_speed"]
        op = MTSynthSampleOp.from_args(target_sample=sample,
                                        target_temperature=reaction_temperature,
                                        target_stirring_speed=reaction_stirring_speed)
        self.request_station_op(op)

    def request_analysis_process(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        proc = APCLCMSAnalysisProcess.from_args(lot=self.lot,
                                                target_batch_index=batch_index,
                                                target_sample_index=sample_index)
        self.request_station_process(proc)

    def request_reaction_stop(self):
        station_op = MTSynthStopReactionOp.from_args()
        self.request_station_op(station_op)

    def request_open_close_drain_valve(self):
        station_op = MTSynthLongOpenCloseReactionValveOp.from_args()
        self.request_station_op(station_op)

    def request_filter_drain(self):
        station_op = APCDrainWasteOp.from_args()
        self.request_station_op(station_op)

    def request_unload_cleaning_funnel(self):
        robot_task = RobotTaskOp.from_args(
            name="unLoadEmptyFunnel",
            target_robot="KMRIIWARobot",
            task_type = 2,
            lbr_program_name = "unLoadEmptyFunnel"
        )
        self.request_robot_ops([robot_task])

    ''' transitions callbacks '''
    def is_reactor_clean(self):
        target_purity_concentration = self.data["target_purity_concentration"]
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, LCMSAnalysisResult):
                return result.concentration <= target_purity_concentration

class APCMeasureYieldProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [
            State(name='init_state'), 
            State(name='prep_state'),
            State(name='navigate_to_weighing_station', on_enter=['request_navigate_to_weighing']),
            State(name='open_sash', on_enter=['request_open_sash']),
            State(name='weight_yield', on_enter=['request_weighing_process']),
            State(name='close_sash', on_enter=['request_close_sash']),
            State(name='final_state')
        ]
            
        ''' Transitions '''
        self.TRANSITIONS = [
            { 'source':'init_state','dest':'prep_state'},
            { 'source':'prep_state','dest':'navigate_to_weighing_station'},
            { 'source':'navigate_to_weighing_station','dest':'open_sash', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'open_sash','dest':'weight_yield', 'conditions':'are_req_station_ops_completed'},
            { 'source':'weight_yield','dest':'close_sash', 'conditions':'are_req_station_procs_completed'},
            { 'source':'close_sash','dest':'final_state', 'conditions':'are_req_station_ops_completed'}
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_batch_index: int,
                  target_sample_index: int,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     APCMetaStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["target_batch_index"] = int(target_batch_index)
        model.data["target_sample_index"] = int(target_sample_index)
        model.save()
        return cls(model)
    
    ''' States callbacks. '''

    def request_navigate_to_weighing(self):
        robot_task = RobotNavOp.from_args(
            name="NavToWeighing",
            target_robot="KMRIIWARobot",
            target_location=None
        )
        wait_for_next_op = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_for_next_op])

    def request_open_sash(self):
        station_op = APCOpenSashOp.from_args()
        self.request_station_op(station_op)

    def request_weighing_process(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        proc = APCWeighingProcess.from_args(lot=self.lot,
                                                target_batch_index=batch_index,
                                                target_sample_index=sample_index)
        self.request_station_process(proc)


    def request_close_sash(self):
        station_op = APCCloseSashOp.from_args()
        self.request_station_op(station_op)
