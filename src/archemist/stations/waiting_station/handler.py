from datetime import datetime, timedelta
from .state import WaitingStation, WaitOp
from archemist.core.processing.handler import SimStationOpHandler
from archemist.core.util.enums import OpOutcome
from archemist.core.state.station_op_result import ProcessOpResult
from typing import List, Tuple


class WaitingStationHandler(SimStationOpHandler):
    def __init__(self, station: WaitingStation):
        super().__init__(station)

    def is_op_execution_complete(self):
        current_op: WaitOp = self._station.assigned_op
        elapsed_time = datetime.now() - current_op.start_timestamp

        if current_op.time_unit == "second":
            wait_duration = timedelta(seconds=current_op.duration)
        elif current_op.time_unit == "minute":
            wait_duration = timedelta(minutes=current_op.duration)
        elif current_op.time_unit == "hour":
            wait_duration = timedelta(hours=current_op.duration)

        return elapsed_time >= wait_duration

    def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
        origin_op: WaitOp = self._station.assigned_op
        parameters = {"duration": origin_op.duration,
                      "time_unit": origin_op.time_unit}
        return OpOutcome.SUCCEEDED, [ProcessOpResult.from_args(origin_op=origin_op.object_id,
                                                               parameters=parameters)]
