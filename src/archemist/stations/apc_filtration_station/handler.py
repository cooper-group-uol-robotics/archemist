from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import ProcessOpResult
from .state import APCDryProductOp
from archemist.core.util.enums import OpOutcome


class SimAPCFiltrationStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
        op = self._station.assigned_op
        if isinstance(op, APCDryProductOp):
            parameters = {}
            parameters["duration"] = op.duration
            parameters["time_unit"] = op.time_unit
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                               parameters=parameters)
            return OpOutcome.SUCCEEDED, [result]
        else:
            return OpOutcome.SUCCEEDED, None
