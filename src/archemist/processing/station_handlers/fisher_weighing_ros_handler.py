import rospy
from typing import Tuple, Dict
from archemist.state.station import Station
from archemist.state.stations.fisher_weighing_station import FisherWeightOpDescriptor
from fisherbrand_pps4102_balance_msgs.msg import BalanceCommand, BalanceReading
from archemist.processing.handler import StationHandler

class FisherWeighingROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._fisher_pu = rospy.Publisher("/Balance_Commands", BalanceCommand, queue_size=2)
        rospy.Subscriber('/Balance_Weights',BalanceReading, self._weight_callback)
        self._received_results = False
        self._op_results = {}
        rospy.sleep(1)

    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')

    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        self._received_results = False
        self._op_results = {}
        if isinstance(current_op,FisherWeightOpDescriptor):
            rospy.loginfo('reading stable weight')
            for i in range(10):
                self._fisher_pu.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results

    def _weight_callback(self, msg: BalanceReading):
        self._received_results = True
        self._op_results['weight'] = msg.weight