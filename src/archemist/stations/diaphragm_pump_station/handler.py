from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult
from .state import DiaphragmPumpDispenseVolumeOp
from archemist.core.util.enums import OpOutcome


class SimDiaphragmPumpStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        if isinstance(op, DiaphragmPumpDispenseVolumeOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[
                                                    op.liquid_name],
                                                amounts=[op.dispense_volume],
                                                units=[op.dispense_unit])

        return OpOutcome.SUCCEEDED, [result]
