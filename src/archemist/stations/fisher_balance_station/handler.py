
from typing import Tuple, List
from archemist.core.state.station import Station
from .state import FisherWeightingStation, FisherWeighOp, FisherWeighResult
from archemist.core.processing.handler import StationOpHandler,  SimStationOpHandler
from archemist.core.util.enums import OpOutcome
import random


class SimFisherWeighingHandler(SimStationOpHandler):
    def __init__(self, station: FisherWeightingStation):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[FisherWeighResult]]:
        current_op = self._station.assigned_op
        result = FisherWeighResult.from_args(origin_op=current_op.object_id,
                                             reading_value=random.random()*100,
                                             unit="mg")
        return OpOutcome.SUCCEEDED, [result]


try:
    import rospy
    from fisherbrand_pps4102_balance_msgs.msg import BalanceCommand, BalanceReading

    class FisherWeighingROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._fisher_pu = rospy.Publisher("/Balance_Commands", BalanceCommand, queue_size=2)
            rospy.Subscriber('/Balance_Weights', BalanceReading, self._weight_callback)
            self._op_result = None
            rospy.sleep(1)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_result = None
            if isinstance(current_op, FisherWeighOp):
                rospy.loginfo('reading stable weight')
                for i in range(10):
                    self._fisher_pu.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            return self._op_result is not None

        def get_op_result(self) -> Tuple[OpOutcome, List[FisherWeighResult]]:
            return OpOutcome.SUCCEEDED, [self._op_result]

        def _weight_callback(self, msg: BalanceReading):
            current_op = self._station.assigned_op
            self._op_result = FisherWeighResult.from_args(origin_op=current_op.object_id,
                                                          reading_value=msg.weight,
                                                          unit="g")  # TODO need to get the unit from driver

        def shut_down(self):
            pass

except ImportError:
    pass
