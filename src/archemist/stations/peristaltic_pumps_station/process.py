from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.lot import Lot
from .state import PeristalticPumpsStation
from archemist.core.state.robot_op import RobotTaskOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import List, Dict, Any

from typing import Union

class PandaPumpSolubilityProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'),
            State(name='move_dispense_head', on_enter='request_move_dispense_head'),
            State(name='dispense_liquid', on_enter='request_dispense_liquid'),
            State(name='replace_dispense_head', on_enter='request_replace_dispense_head'),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'move_dispense_head'},
            {'source':'move_dispense_head','dest':'dispense_liquid', 'conditions':'are_req_robot_ops_completed'},
            {'source':'dispense_liquid','dest':'replace_dispense_head', 'conditions':'are_req_station_ops_completed'},
            {'source':'replace_dispense_head','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
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
                                     PeristalticPumpsStation.__name__,
                                     lot,
                                     operations,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)

    ''' states callbacks '''

    def request_move_dispense_head(self):
        robot_op = RobotTaskOp.from_args(name='move_dispense_head', target_robot="PandaRobot")
        
        self.request_robot_ops([robot_op])

    def request_dispense_liquid(self):
        sample = self.lot.batches[0].samples[0]
        current_op = self.generate_operation("dispense_op", target_sample=sample)
        self.request_station_op(current_op)

    def request_replace_dispense_head(self):
        robot_op = RobotTaskOp.from_args(name='replace_dispense_head', target_robot="PandaRobot")
        self.request_robot_ops([robot_op])