from typing import Tuple, List
import time
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler, StationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import (
    MTSynthHeatStirOp,
    MTSynthSampleOp,
    MTSynthCustomOpenCloseReactionValveOp,
    MTSynthStopReactionOp,
    MTSynthLongOpenCloseReactionValveOp,
    MTSynthesisStation,
    MTSynthWaitOp
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
        elif isinstance(op, MTSynthWaitOp):
                rospy.loginfo("Waiting (crystallisation).")
                time.sleep(op.seconds)

        return OpOutcome.SUCCEEDED, [result] if result is not None else None

try:
    import rospy
    from roslabware_msgs.msg import MettlerOptimaxCmd, MettlerOptimaxReading, MettlerOptimaxTask, BaseValveCmd, BaseValveStatus, BaseValveTask
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
                "/mettler_optimax/task_complete", MettlerOptimaxTask, self.MTOptimax_callback
            )
            rospy.Subscriber(
                "/base_valve/task_complete", BaseValveTask, self.BaseValve_callback
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
                        optimax_command = MettlerOptimaxCmd.HEAT_WAIT,
                        temperature = current_op.target_temperature,
                        stir_speed = current_op.target_stirring_speed,
                        wait_duration = current_op.wait_duration,
                    )

            elif isinstance(current_op, MTSynthSampleOp):
                rospy.loginfo(
                    f"Sampling operation started"
                )
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        optimax_command = MettlerOptimaxCmd.SAMPLE,
                        temperature = current_op.target_temperature,
                        stir_speed = current_op.target_stirring_speed,
                        dilution = current_op.dilution
                    )

            elif isinstance(current_op, MTSynthStopReactionOp):
                rospy.loginfo(f"Synthesis operation stopped")
                for i in range(10):
                    self._pub_optimax.publish(
                        seq=self._seq_id,
                        optimax_command = MettlerOptimaxCmd.STOP
                    )

            elif isinstance(current_op, MTSynthCustomOpenCloseReactionValveOp):
                rospy.loginfo(f"Base valve open & close with custom number of steps.")
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                        valve_command = BaseValveCmd.UPDATE,
                        num_steps = current_op.steps
                    )
                time.sleep(5)
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                        valve_command = BaseValveCmd.OPEN_CLOSE_STEPS
                    )

            elif isinstance(current_op, MTSynthLongOpenCloseReactionValveOp):
                rospy.loginfo(f"Base valve open & close for a long time.")
                for i in range(10):
                    self._pub_base_valve.publish(
                        seq=self._seq_id, 
                        valve_command = BaseValveCmd.OPEN_CLOSE_LONG
                    )

            elif isinstance(current_op, MTSynthWaitOp):
                rospy.loginfo("Waiting (crystallisation).")
                time.sleep(current_op.seconds)

            else:
                rospy.logwarn(
                    f"[{self.__class__.__name__}] Unkown operation was received"
                )

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

        def MTOptimax_callback(self, msg:MettlerOptimaxTask):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1
        
        def BaseValve_callback(self, msg:BaseValveTask):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1

except ImportError:
    pass
