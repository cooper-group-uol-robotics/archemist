from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import (MTSynthDispenseLiquidOp,
                    MTSynthDispenseSolidOp,
                    MTSynthReactAndWaitOp,
                    MTSynthReactAndSampleOp)
from archemist.core.util.enums import OpOutcome

class SimMTSynthesisStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        result = None
        if isinstance(op, MTSynthDispenseLiquidOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                        material_names=[op.liquid_name],
                                                        amounts=[op.dispense_volume],
                                                        units=[op.dispense_unit])
        elif isinstance(op, MTSynthDispenseSolidOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                        material_names=[op.solid_name],
                                                        amounts=[op.dispense_mass],
                                                        units=[op.dispense_unit])
        elif isinstance(op, MTSynthReactAndWaitOp):
            parameters = {}
            parameters["target_temperature"]  = op.target_temperature
            parameters["target_stirring_speed"]  = op.target_stirring_speed
            parameters["wait_duration"]  = op.wait_duration
            parameters["time_unit"]  = op.time_unit
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                parameters=parameters)
        elif isinstance(op, MTSynthReactAndSampleOp):
            parameters = {}
            parameters["target_temperature"]  = op.target_temperature
            parameters["target_stirring_speed"]  = op.target_stirring_speed
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                parameters=parameters)
        
        return OpOutcome.SUCCEEDED, [result] if result is not None else None