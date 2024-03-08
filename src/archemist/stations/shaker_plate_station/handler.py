
from typing import Tuple, List
from archemist.core.state.station import Station
from .state import ShakerPlateOp
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.util.enums import OpOutcome
from archemist.core.state.station_op_result import ProcessOpResult


class SimShakePlateHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
        current_op = self._station.assigned_op
        parameters = {}
        if isinstance(current_op, ShakerPlateOp):
            parameters["duration"] = current_op.duration
            parameters["time_unit"] = current_op.time_unit
        op_result = ProcessOpResult.from_args(origin_op=current_op.object_id,
                                              parameters=parameters)
        return OpOutcome.SUCCEEDED, [op_result]


try:
    import rospy
    from shaker_plate_msgs.msg import ShakerCommand, ShakerStatus

    class ShakePlateROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)

        def initialise(self) -> bool:
            self._waiting_for = False
            self._task_seq = 0
            self._start_time = -1
            self._task_finished = False
            rospy.init_node(f'{self._station}_handler')
            self._shaker_plate_pu = rospy.Publisher(
                "/shaker_plate/command", ShakerCommand, queue_size=1)
            rospy.Subscriber('/shaker_plate/status', ShakerStatus,
                             self._state_update, queue_size=1)
            rospy.sleep(1)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            if (isinstance(current_op, ShakerPlateOp)):
                rospy.loginfo('sending shaking command')
                rospy.loginfo(f'===>{current_op.duration}')
                self._start_time = rospy.get_time()
                self._waiting_for = True
                self._task_finished = False
                self._task_seq += 1
                if current_op.time_unit == "second":
                    self.total_seconds = current_op.duration,
                elif current_op.time_unit == "minute":
                    self.total_seconds = current_op.duration, *60
                elif current_op.time_unit == "hour":
                    self.total_seconds = current_op.duration, *60*60
                msg = ShakerCommand(
                    shake_duration=current_op.self.total_seconds, task_seq=self._task_seq)
                for i in range(10):
                    self._shaker_plate_pu.publish(msg)
            else:
                rospy.logwarn(
                    f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            return self._task_finished

        def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
            current_op = self._station.assigned_op
            parameters = {}
            if isinstance(current_op, ShakerPlateOp):
                parameters["duration"] = current_op.duration
                parameters["time_unit"] = current_op.time_unit
            op_result = ProcessOpResult.from_args(origin_op=current_op.object_id,
                                                  parameters=parameters)
            return OpOutcome.SUCCEEDED, [op_result]

        def _state_update(self, msg: ShakerStatus):
            if self._waiting_for:
                elepsed_duration = rospy.get_time() - self._start_time
                if elepsed_duration >= self.total_seconds and msg.status == ShakerStatus.NOT_SHAKING:
                    self._task_finished = True
                    self._waiting_for = False

except ImportError:
    pass
