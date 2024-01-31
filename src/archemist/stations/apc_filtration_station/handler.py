from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler, StationOpHandler
from archemist.core.state.station_op_result import ProcessOpResult
from .state import APCFiltrationStation, APCDryProductOp, APCFilterProductOp, APCDrainWasteOp
from archemist.core.util.enums import OpOutcome

class SimAPCFiltrationStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
        op = self._station.assigned_op
        if isinstance(op, APCDryProductOp):
            parameters = {}
            parameters["duration"]  = op.duration
            parameters["time_unit"]  = op.time_unit
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                parameters=parameters)
            return OpOutcome.SUCCEEDED, [result]
        else:
            return OpOutcome.SUCCEEDED, None


try:
    import rospy
    from roslabware_msgs.msg import FiltrationCmd, FiltrationStatus, FiltrationTask
    from std_msgs.msg import Bool

    class APCFiltrationStationHandler(StationOpHandler):
        def __init__(self, station: APCFiltrationStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f"{self._station}_handler")
            self._pub_optimax = rospy.Publisher(
                "/filtration_command", FiltrationCmd, queue_size=2
            )

            rospy.Subscriber(
                "/filtration/task_complete", FiltrationTask, self.Filtration_callback
            )

            self._op_complete = False
            self._op_results = {}
            self._seq_id = 1
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, APCFilterProductOp):
                rospy.loginfo(
                    f"Filtration operation started"
                )

                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        filtration_system_command = FiltrationCmd.MAIN_FILTRATION,
                    )

            elif isinstance(current_op, APCDryProductOp):
                rospy.loginfo(
                    f"Drying operation started"
                )
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        filtration_system_command = FiltrationCmd.DRY,
                    )

            elif isinstance(current_op, APCDrainWasteOp):
                rospy.loginfo(f"Drain operation stopped")
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        filtration_system_command = FiltrationCmd.TIMED_DRAIN
                    )

            else:
                rospy.logwarn(
                    f"[{self.__class__.__name__}] Unkown operation was received"
                )

        def is_op_execution_complete(self) -> bool:
            return self._op_complete

        def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
            op = self._station.assigned_op
            if isinstance(op, APCDryProductOp):
                parameters = {}
                parameters["duration"]  = op.duration
                parameters["time_unit"]  = op.time_unit
                result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                    parameters=parameters)
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

        def shut_down(self):
            pass

        def Filtration_callback(self, msg):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1
        

except ImportError:
    pass
