from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import APCSPumpDispenseVolumeOp, APCSPumpDispenseRateOp,APCSPumpFinishDispensingOp
from archemist.core.util.enums import OpOutcome
from random import random

class SimAPCSyringePumpStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        if isinstance(op, APCSPumpDispenseVolumeOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.liquid_name],
                                                amounts=[op.dispense_volume],
                                                units=[op.dispense_unit])
        elif isinstance(op, APCSPumpFinishDispensingOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.liquid_name],
                                                amounts=[random()*5],
                                                units=["mL"])
        elif isinstance(op, APCSPumpDispenseRateOp):
            parameters = {
                "dispense_rate": op.dispense_rate,
                "rate_unit": op.rate_unit
            }
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                parameters=parameters)

        return OpOutcome.SUCCEEDED, [result]