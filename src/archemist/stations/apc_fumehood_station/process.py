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
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 
            
            State(name='add_solid', on_enter=['request_adding_solid']),

            State(name='update_cartridge_state', on_enter=['request_update_cartridge_state']),

            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'add_solid'},
            {'source':'add_solid','dest':'update_cartridge_state', 'conditions':'are_req_station_ops_completed'},
            {'source':'update_cartridge_state','dest':'final_state'},
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

    def request_adding_solid(self):
        synth_station: APCFumehoodStation = self.get_assigned_station()
        params_dict = {}
        for cartridge in synth_station.cartridges:
            if not cartridge.depleted:
                cartridge_index = cartridge.hotel_index
                break
        robot_task = RobotTaskOp.from_args(
            name="addSolid",
            target_robot="KMRIIWARobot",
            task_type = 2,
            lbr_program_name = "addSolid",
            lbr_program_params = [str(cartridge_index)]
            )
        wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, wait_task])
    
    def request_update_cartridge_state(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        current_op = self.generate_operation("add_solid", target_sample=sample)
        self.request_station_op(current_op)
