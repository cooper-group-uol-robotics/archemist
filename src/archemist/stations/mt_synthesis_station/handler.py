from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler, StationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import (
    MTSynthHeatStirOp,
    MTSynthSampleOp,
    MTSynthTimedOpenReactionValveOp,
    MTSynthStopReactionOp,
    MTSynthCloseReactionValveOp,
    MTSynthOpenReactionValveOp,
    MTSynthesisStation,
)
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
            result = ProcessOpResult.from_args(
                origin_op=op.object_id, parameters=parameters
            )
        elif isinstance(op, MTSynthSampleOp):
            parameters = {}
            parameters["target_temperature"] = op.target_temperature
            parameters["target_stirring_speed"] = op.target_stirring_speed
            result = ProcessOpResult.from_args(
                origin_op=op.object_id, parameters=parameters
            )

        return OpOutcome.SUCCEEDED, [result] if result is not None else None

try:
    import rospy
    from roslabware_msgs.msg import MettlerOptimaxCmd, MettlerOptimaxReading, BaseValveCmd, BaseValveStatus
    from std_msgs.msg import Bool

    class APCMTSynthesisStationRosHandler(StationOpHandler):
        def __init__(self, station: MTSynthesisStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f"{self._station}_handler")
            self._pub_optimax = rospy.Publisher(
                "/mettler_optimax", MettlerOptimaxCmd, queue_size=2
            )
            self._pub_base_valve = rospy.Publisher(
                "/optimax_basevalve_command", BaseValveCmd, queue_size=2
            )
            rospy.Subscriber(
                "/mettler_optimax/task_complete", Bool, self.MTOptimax_callback
            )
            rospy.Subscriber(
                "/base_valve/task_complete", Bool, self.BaseValve_callback
            )
            self._op_complete = False
            self._op_results = {}
            self._seq_id = 1
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, MTSynthHeatStirOp):
                rospy.loginfo(
                    f"Synthesis operation started"
                )

                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        optimax_command = MettlerOptimaxCmd.PARA_HW,
                        temperature = current_op.target_temperature,
                        stir_speed = current_op.target_stirring_speed,
                        wait_duration = current_op.wait_duration
                    )

            elif isinstance(current_op, MTSynthSampleOp):
                rospy.loginfo(
                    f"Sampling operation started"
                )
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        optimax_command = MettlerOptimaxCmd.PARA_S,
                        temperature = current_op.target_temperature,
                        stir_speed = current_op.target_stirring_speed
                    )

            elif isinstance(current_op, MTSynthStopReactionOp):
                rospy.loginfo(f"Synthesis operation stopped")
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        optimax_command = MettlerOptimaxCmd.STOP
                    )

            elif isinstance(current_op, MTSynthTimedOpenReactionValveOp):
                rospy.loginfo(f"Base valve timed open & close")
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                    )

            elif isinstance(current_op, MTSynthOpenReactionValveOp):
                rospy.loginfo(f"Base valve open")
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                        valve_command = BaseValveCmd.OPEN
                    )

            elif isinstance(current_op, MTSynthCloseReactionValveOp):
                rospy.loginfo(f"Base valve close")
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                        valve_command = BaseValveCmd.CLOSE
                    )
            else:
                rospy.logwarn(
                    f"[{self.__class__.__name__}] Unkown operation was received"
                )
            self._seq_id += 1

        def is_op_execution_complete(self) -> bool:
            return self._op_complete

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
                result = ProcessOpResult.from_args(
                    origin_op=op.object_id, parameters=parameters
                )
            elif isinstance(op, MTSynthSampleOp):
                parameters = {}
                parameters["target_temperature"] = op.target_temperature
                parameters["target_stirring_speed"] = op.target_stirring_speed
                result = ProcessOpResult.from_args(
                    origin_op=op.object_id, parameters=parameters
                )

            return OpOutcome.SUCCEEDED, [result] if result is not None else None

        def shut_down(self):
            pass

        def MTOptimax_callback(self, msg):
            self._op_complete = msg.data
        
        def BaseValve_callback(self, msg):
            self._op_complete = msg.data

except ImportError:
    pass
