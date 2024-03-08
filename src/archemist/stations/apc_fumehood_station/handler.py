from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult
from .state import APCDispenseSolidOp
from archemist.core.util.enums import OpOutcome


class SimAPCFumehoodStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        result = None
        if isinstance(op, APCDispenseSolidOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.solid_name],
                                                amounts=[op.dispense_mass],
                                                units=[op.dispense_unit])
        else:
            result = None

        return OpOutcome.SUCCEEDED, [result] if result is not None else None
