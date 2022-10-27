import rospy
from archemist.core.state.station import Station
from .state import IKAHeatingOpDescriptor, IKAStirringOpDescriptor, IKAHeatingStirringOpDescriptor
from archemist.core.processing.handler import StationHandler
from ika_plate_rct_digital_msgs.msg import IKACommand
from threading import Thread
from typing import Tuple, Dict

class IKAPlateDigitalROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._ika_pub = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        self._timer_thread = None
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

        if isinstance(current_op, IKAHeatingOpDescriptor):
            rospy.loginfo("executing heating operation")
            for i in range(10):
                self._ika_pub.publish(ika_command= IKACommand.HEATAT, ika_param=current_op.target_temperature)
        elif isinstance(current_op, IKAStirringOpDescriptor):
            rospy.loginfo("executing stirring operation")
            for i in range(10):
                self._ika_pub.publish(ika_command= IKACommand.STIRAT, ika_param=current_op.target_stirring_speed)
        elif isinstance(current_op, IKAHeatingStirringOpDescriptor):
            rospy.loginfo("executing heating and stirring operation")
            for i in range(10):
                self._ika_pub.publish(ika_command= IKACommand.HEATAT, ika_param=current_op.target_temperature)
                self._ika_pub.publish(ika_command= IKACommand.STIRAT, ika_param=current_op.target_stirring_speed)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')
        
        self._timer_thread = Thread(target=self._sleep_for_duration,kwargs={'duration':current_op.target_duration})
        self._timer_thread.start()


    def is_op_execution_complete(self) -> bool:
        if self._timer_thread.is_alive():
            return False
        else:
            return True

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}

    def _sleep_for_duration(self, **kwargs):
        rospy.sleep(kwargs['duration'])
        for i in range(10):
            self._ika_pub.publish(ika_command= IKACommand.ALLOFF)
