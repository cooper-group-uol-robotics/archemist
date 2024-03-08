import rospy
from typing import Tuple, List, Optional
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from .state import (
    APCWeighingStation,
    APCOpenBalanceDoorOp,
    APCCloseBalanceDoorOp,
    APCTareOp,
    APCWeighingOp,
    APCWeighResult
)
from archemist.core.util.enums import OpOutcome


class SimAPCWeighingStationHandler(SimStationOpHandler):
    def __init__(self, station: APCWeighingStation):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, Optional[List[APCWeighResult]]]:
        current_op = self._station.assigned_op
        if isinstance(current_op, APCWeighingOp):
            result = APCWeighResult.from_args(
                origin_op=current_op.object_id,
                reading_value=42
            )
            return OpOutcome.SUCCEEDED, [result]
        else:
            return OpOutcome.SUCCEEDED, None


try:
    import rospy
    from roslabware_msgs.msg import KernPCB2500Cmd, KernPCB2500Reading
    from roslabware_msgs.msg import KernDoorCmd, KernDoorStatus
    from roslabware_msgs.msg import sashDoorCmd, sashDoorStatus

    class APCWeighingStationHandler(StationOpHandler):
        def __init__(self, station: APCWeighingStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._pub_balance = rospy.Publisher(
                "kern_PCB2500_Commands", KernPCB2500Cmd, queue_size=2)
            self._pub_door = rospy.Publisher(
                "kern_door_Commands", KernDoorCmd, queue_size=2)
            self._pub_sash = rospy.Publisher(
                "sash_door_Commands", sashDoorCmd, queue_size=2)
            rospy.Subscriber("kern_PCB2500_Readings",
                             KernPCB2500Reading, self.weight_callback)
            rospy.Subscriber("kern_Door_Status",
                             KernDoorStatus, self.door_callback)
            rospy.Subscriber("sash_door_Status",
                             sashDoorStatus, self.sash_callback)
            self._target_balance_door_status = None
            self._target_sash_door_status = None
            self._received_mass = False
            self._op_results = {}
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._command_executed = False
            if isinstance(current_op, APCWeighingOp):
                self.read_weight = None
                rospy.loginfo('Reading stable weight.')
                for i in range(10):
                    self._pub_balance.publish(
                        kern_command=KernPCB2500Cmd.GET_MASS_STABLE)
            elif isinstance(current_op, APCTareOp):
                rospy.loginfo('Taring balance.')
                for i in range(10):
                    self._pub_balance.publish(
                        kern_command=KernPCB2500Cmd.TARE_BALANCE)
            elif isinstance(current_op, APCOpenBalanceDoorOp):
                rospy.loginfo('Opening balance door.')
                self._target_balance_door_status = "door_open"
                for i in range(10):
                    self._pub_door.publish(
                        kern_door_command=KernDoorCmd.OPEN_DOOR)
            elif isinstance(current_op, APCCloseBalanceDoorOp):
                rospy.loginfo('Closing balance door.')
                self._target_balance_door_status = "door_closed"
                for i in range(10):
                    self._pub_door.publish(
                        kern_door_command=KernDoorCmd.CLOSE_DOOR)
            else:
                rospy.logwarn(
                    f'[{self.__class__.__name__}] Unkown operation was received')

        # TODO not sure what this is doing
        def is_op_execution_complete(self) -> bool:
            return self._command_executed

        def get_op_result(self) -> Tuple[OpOutcome, Optional[List[APCWeighResult]]]:
            current_op = self._station.assigned_op
            if isinstance(current_op, APCWeighingOp):
                result = APCWeighResult.from_args(
                    origin_op=current_op.object_id,
                    reading_value=self.read_weight)
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

        def shut_down(self):
            pass

        def weight_callback(self, msg):
            self._command_executed = True
            self.read_weight = msg.mass

        def door_callback(self, msg):
            if self._target_balance_door_status == str(msg.status):
                self._command_executed = True

        def sash_callback(self, msg):
            if self._target_sash_door_status == str(msg.status):
                self._command_executed = True

except ImportError:
    pass
