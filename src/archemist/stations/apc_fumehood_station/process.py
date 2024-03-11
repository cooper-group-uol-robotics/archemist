from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp
from .state import APCFumehoodStation, APCDispenseSolidOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union, List, Dict, Any
from archemist.core.util import Location

class APCSolidAdditionProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 
            State(name='load_cartridge', on_enter=['set_station_loaded_cartridge']),
            State(name='add_solid', on_enter=['request_adding_solid']),
            State(name='update_cartridge_state', on_enter=['request_update_cartridge_state']),
            State(name='unload_cartridge', on_enter=['unset_station_loaded_cartridge']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'load_cartridge'},
            {'source':'load_cartridge', 'dest': 'add_solid', 'conditions':'are_req_station_ops_completed'},
            {'source':'add_solid','dest':'update_cartridge_state', 'conditions':'are_req_robot_ops_completed'},
            {'source':'update_cartridge_state', 'dest': 'unload_cartridge', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_cartridge','dest':'final_state'},
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

    def set_station_loaded_cartridge(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        for index, cartridge in enumerate(synth_station.cartridges):
            if not cartridge.depleted:
                synth_station.load_cartridge(index)
                break

    def request_adding_solid(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        for cartridge in synth_station.cartridges:
            if not cartridge.depleted:
                cartridge_index = cartridge.hotel_index
                break
        location_dict = {"coordinates": [35, 8], "descriptor": "Solid addition"}
        target_loc = Location.from_dict(location_dict)
        robot_task = RobotTaskOp.from_args(
            name="addSolid",
            target_robot="KMRIIWARobot",
            target_location=target_loc,
            params={},  
            lbr_program_name="addSolid",
            lbr_program_params=[str(cartridge_index)],
            fine_localization=True,
            task_type=2
        )
        self.request_robot_ops([robot_task])
    
    def request_update_cartridge_state(self):
        # Update the solid addition cartridge - uses APCDispenseSolidOp
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("add_solid", target_sample=sample)
        self.request_station_op(current_op)

    def unset_station_loaded_cartridge(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        synth_station.unload_cartridge()
