from typing import List, Tuple
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station import Station
from archemist.core.util.enums import OpOutcome
from .state import SolubilityOpResult, CheckSolubilityOp, SolubilityState
import random


class SimSolubilityStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[SolubilityOpResult]]:
        current_op = self._station.assigned_op
        if isinstance(current_op, CheckSolubilityOp):
            solubility_state = random.choice(
                [SolubilityState.DISSOLVED, SolubilityState.UNDISSOLVED])
            result = SolubilityOpResult.from_args(origin_op=current_op.object_id,
                                                  solubility_state=solubility_state,
                                                  result_filename=f"some_file_{random.randint(1,100)}.png")
        return OpOutcome.SUCCEEDED, [result]
