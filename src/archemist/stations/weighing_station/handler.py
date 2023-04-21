import rospy
from typing import Dict, Tuple
from archemist.core.processing.handler import StationHandler
from archemist.core.state.station import Station
from kern_pcb_balance_msgs.msg import KernCommand, KernReading
from .state import SampleWeighingOpDescriptor

class KernPcbROSHandler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubBalanceTare = rospy.Publisher("Kern_Commands", KernCommand, queue_size=2)
        rospy.Subscriber("Kern_Weights", KernReading, self.weight_callback)
        for i in range(10):
            self.pubBalanceTare.publish(kern_command = KernCommand.ZERO)
        self._received_results = False
        self._op_results = 0.0
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
        if isinstance(current_op, SampleWeighingOpDescriptor):
            for i in range(25):
                self.pubBalanceTare.publish(kern_command = KernCommand.GET_MASS)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results

    def weight_callback(self, msg):
        if msg.weight > 0:
            self._op_results['weight'] = msg.weight
            rospy.loginfo(f'The weight of the funnel is [{self._op_results}]')
            self._received_results = True
            # for i in range(10):
            #     self.pubBalanceTare.publish(kern_command = KernCommand.ZERO)
        else:
            pass


    
