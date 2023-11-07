from transitions import State
from archemist.core.state.lot import Lot
from .state import IKADigitalPlateStation
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import DropBatchOpDescriptor, RobotTaskOpDescriptor
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, Dict, Any, List
from datetime import datetime, timedelta


class PXRDWorkflowStirringProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 
            State(name='place_lot', on_enter=['request_place_lot']),
            State(name='load_stir_plate', on_enter=['request_load_stir_plate']),
            State(name='stir', on_enter=['request_stir_op']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'place_lot'},
            {'source':'place_lot', 'dest': 'load_stir_plate', 'conditions':'are_req_robot_ops_completed'},
            {'source':'load_stir_plate','dest':'stir', 'conditions':'are_req_robot_ops_completed'},
            {'source':'stir','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

        if self.data["eight_well_rack_first"]:
            self.eight_well_batch_index = 0
            self.pxrd_batch_index = 1
        else:
            self.eight_well_batch_index = 1
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
                                     IKADigitalPlateStation.__name__,
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.data["eight_well_rack_first"] = eight_well_rack_first
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_place_lot(self):
        # place the 8-well rack
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        params_dict = {}
        params_dict["perform_6p_calib"] = True
        robot_op_1 = DropBatchOpDescriptor.from_args(name='LoadEightWRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_1)
        # place the pxrd rack
        batch_2 = self.lot.batches[self.pxrd_batch_index]
        params_dict["perform_6p_calib"] = False
        robot_op_2 = DropBatchOpDescriptor.from_args(name='LoadPXRDRackYumiStation', target_robot="KMRIIWARobot",
                                                       params=params_dict, target_batch=batch_2)
        
        self.request_robot_ops([robot_op_1, robot_op_2])

    def request_load_stir_plate(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        robot_op = RobotTaskOpDescriptor.from_args(name='loadIKAPlate', target_robot="YuMiRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_stir_op(self):
        batch_1 = self.lot.batches[self.eight_well_batch_index]
        current_op = self.generate_operation("stir_op", target_batch=batch_1)
        self.request_station_op(current_op)

class PandaIKASolubilityProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='load_ika_plate', on_enter='request_load_ika_plate'),
            State(name='start_stirring_heating', on_enter='request_stir_heat_op'),
            State(name='check_solubility', on_enter='request_check_solubility'),
            State(name='liquid_addition', on_enter='request_liquid_addition'),
            State(name='sleep', on_enter='request_sleep'),
            State(name='stop_stirring_heating', on_enter='request_ika_stop'),
            State(name='unload_ika_plate', on_enter='request_unload_ika_plate'),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'load_ika_plate'},
            {'source':'load_ika_plate','dest':'start_stirring_heating', 'conditions':'are_req_robot_ops_completed'},
            {'source':'start_stirring_heating','dest':'sleep', 'conditions':'are_req_station_ops_completed'},
            {'source':'sleep','dest':'check_solubility', 'conditions':'is_checking_time'},
            {'source':'check_solubility','dest':'liquid_addition', 'unless':'is_solid_dissolved' ,'conditions':'are_req_station_procs_completed'},
            {'source':'liquid_addition','dest':'sleep', 'conditions':'are_req_station_procs_completed'},
            {'source':'check_solubility','dest':'stop_stirring_heating', 'conditions':['are_req_station_procs_completed', 'is_solid_dissolved']},
            {'source':'stop_stirring_heating','dest':'unload_ika_plate', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_ika_plate','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  operations: List[Dict[str, Any]] = None,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     IKADigitalPlateStation.__name__,
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['start_time'] = None

    def request_load_ika_plate(self):
        batch = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='load_ika_plate', target_robot="PandaRobot",
                                                   target_batch=batch)
        
        self.request_robot_ops([robot_op])

    def request_stir_heat_op(self):
        batch = self.lot.batches[0]
        current_op = self.generate_operation("stir_heat_op", target_batch=batch)
        self.request_station_op(current_op)

    def request_ika_stop(self):
        current_op = self.generate_operation("stop_op")
        self.request_station_op(current_op)

    def request_unload_ika_plate(self):
        batch = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='unload_ika_plate', target_robot="PandaRobot",
                                                   target_batch=batch)
        
        self.request_robot_ops([robot_op])

    def request_check_solubility(self):
        from archemist.stations.solubility_station.process import PandaCheckSolubilityProcess
        operations = [
                {
                    "name": "check_solubility_op",
                    "op": "CheckSolubilityOp",
                    "parameters": None
                }
            ]
        ext_process = PandaCheckSolubilityProcess.from_args(self.lot,operations=operations)
        self.request_station_process(ext_process)

    def request_liquid_addition(self):
        from archemist.stations.peristaltic_pumps_station.process import PandaPumpSolubilityProcess
        operations = [
                {
                    "name": "dispense_op",
                    "op": "PPLiquidDispenseOp",
                    "parameters": {
                        "liquid_name": "water",
                        "dispense_volume": 10,
                        "dispense_unit": "mL"
                    }
                }
            ]
        ext_process = PandaPumpSolubilityProcess.from_args(self.lot,operations=operations)
        self.request_station_process(ext_process)

    def request_sleep(self):
        self.data["start_time"] = datetime.now().isoformat()

    ''' transitions callbacks '''
    def is_checking_time(self):
        start_time = datetime.fromisoformat(self.data["start_time"])
        current_time = datetime.now()
        if current_time - start_time >= timedelta(seconds=1):
            return True
        else:
            return False

    def is_solid_dissolved(self):
        from archemist.stations.solubility_station.state import SolubilityOpResult, SolubilityState
        sample = self.lot.batches[0].samples[0]
        results = list(sample.result_ops)
        for result in reversed(results):
            if isinstance(result, SolubilityOpResult):
                return result.solubility_state == SolubilityState.DISSOLVED