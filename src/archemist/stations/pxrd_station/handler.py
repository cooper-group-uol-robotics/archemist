from typing import Tuple, Optional, List
from archemist.core.state.station import Station
from .state import PXRDAnalysisOp, PXRDAnalysisResult
from archemist.core.processing.handler import StationHandler, SimStationOpHandler
from archemist.core.util.enums import OpOutcome
from random import randint


class SimPXRDStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, Optional[List[PXRDAnalysisResult]]]:
        current_op = self._station.assigned_op
        if isinstance(current_op, PXRDAnalysisOp):
            result = PXRDAnalysisResult.from_args(origin_op=current_op.object_id,
                                                  result_filename=f"pxrd_result_{randint(1, 100)}.xml")
            return OpOutcome.SUCCEEDED, [result]
        else:
            return OpOutcome.SUCCEEDED, None


try:
    import rospy
    # from std_msgs.msg import String
    from pxrd_msgs.msg import PxrdCommand, PxrdStatus

    class PXRDStationROSHandler(StationHandler):
        def __init__(self, station: Station):
            super().__init__(station)

        def initialise(self) -> bool:
            self._current_pxrd_status = PxrdStatus.NOT_LAUNCHED_YET
            self._desired_pxrd_status = None
            rospy.init_node(f'{self._station}_handler')
            self.pub_pxrd = rospy.Publisher(
                "/PXRD_commands", PxrdCommand, queue_size=1)
            rospy.Subscriber('/PXRD_status', PxrdStatus,
                             self._pxrd_state_update, queue_size=1)
            rospy.sleep(1)
            if self._current_pxrd_status:
                return True
            else:
                return False

        def execute_op(self):
            current_op = self._station.assigned_op
            print(f'performing pxrd operation {current_op}')
            if isinstance(current_op, PXRDAnalysisOp):
                rospy.loginfo('starting pxrd job')
                for i in range(10):
                    self.pub_pxrd.publish(PXRD_commands=PxrdCommand.EXECUTE)
                self._desired_pxrd_status = PxrdStatus.EXECUTION_DONE

        def is_op_execution_complete(self):
            if self._desired_pxrd_status == self._current_pxrd_status:
                self._desired_pxrd_status = None
                return True
            else:
                return False

        def get_op_result(self) -> Tuple[OpOutcome, Optional[List[PXRDAnalysisResult]]]:
            current_op = self._station.assigned_op
            if isinstance(current_op, PXRDAnalysisOp):
                result = PXRDAnalysisResult.from_args(origin_op=current_op.object_id,
                                                      result_filename="somefile.xml")  # TODO need to be implmented in ROS message
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

        def _pxrd_state_update(self, msg):
            if self._current_pxrd_status != msg.pxrd_status:
                self._current_pxrd_status = msg.pxrd_status


except ImportError:
    pass
