from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import SimStationOpHandler, StationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult
from .state import APCDispenseSolidOp, APCCloseSashOp, APCOpenSashOp, APCFumehoodStation, APCCartridge
from archemist.core.util.enums import OpOutcome

class SimAPCFumehoodStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        result = None
        if isinstance(op, APCDispenseSolidOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.solid_name],
                                                amounts=[op.dispense_mass],
                                                units=[op.dispense_unit])
        else:
            result = None
        
        return OpOutcome.SUCCEEDED, [result] if result is not None else None

try:
    import rospy
    from roslabware_msgs.msg import SashDoorCmd, SashDoorStatus, SashDoorTask
    from std_msgs.msg import Bool

    class APCFumeHoodStationRosHandler(StationOpHandler):
        def __init__(self, station: APCFumehoodStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f"{self._station}_handler")
            self._pub_sashDoor = rospy.Publisher(
                "/sash_door_command", SashDoorCmd, queue_size=2
            )
            
            rospy.Subscriber(
                "/sash_door/task_complete", SashDoorTask, self.SashDoor_callback
            )
            self._op_complete = False
            self._op_results = {}
            self._seq_id = 1
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, APCOpenSashOp):
                rospy.loginfo(
                    f"Sash door open operation started"
                )

                for i in range(10):
                    self._pub_sashDoor.publish(
                        seq=self._seq_id,
                        sash_door_command = SashDoorCmd.OPEN_DOOR,
                    )

            elif isinstance(current_op, APCCloseSashOp):
                rospy.loginfo(
                    f"Sash door close operation started"
                )
                for i in range(10):
                    self._pub_sashDoor.publish(
                        seq=self._seq_id,
                        sash_door_command = SashDoorCmd.CLOSE_DOOR
                    )
            else:
                rospy.logwarn(
                    f"[{self.__class__.__name__}] Unkown operation was received"
                )

        def is_op_execution_complete(self) -> bool:
            return self._op_complete

        def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            result = None
            if isinstance(op, APCDispenseSolidOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                    material_names=[op.solid_name],
                                                    amounts=[op.dispense_mass],
                                                    units=[op.dispense_unit])
            else:
                result = None
            
            return OpOutcome.SUCCEEDED, [result] if result is not None else None

        def shut_down(self):
            pass

        def SashDoor_callback(self, msg:SashDoorTask):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1

except ImportError as e:
    print(f"Error in handler: {e}")
    pass
