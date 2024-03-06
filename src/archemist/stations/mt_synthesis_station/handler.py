from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import (MTSynthHeatStirOp,
                    MTSynthSampleOp)
from archemist.core.util.enums import OpOutcome


class SimMTSynthesisStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        result = None
        if isinstance(op, MTSynthHeatStirOp):
            parameters = {}
            parameters["target_temperature"] = op.target_temperature
            parameters["target_stirring_speed"] = op.target_stirring_speed
            if op.wait_duration is not None:
                parameters["wait_duration"] = op.wait_duration
                parameters["time_unit"] = op.time_unit
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                               parameters=parameters)
        elif isinstance(op, MTSynthSampleOp):
            parameters = {}
            parameters["target_temperature"] = op.target_temperature
            parameters["target_stirring_speed"] = op.target_stirring_speed
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                               parameters=parameters)

        return OpOutcome.SUCCEEDED, [result] if result is not None else None
