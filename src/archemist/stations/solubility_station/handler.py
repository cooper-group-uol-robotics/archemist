import rospy
from typing import Dict, Tuple
from archemist.core.processing.handler import StationHandler
from archemist.core.state.station import Station
from .state import SolubilityOpDescriptor
from archemist_msgs.msg import CameraCommand

from rospy.core import is_shutdown

class SolubilityStationROSHandler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._camera_pub = rospy.Publisher("/camera1/commands", CameraCommand, queue_size=2)
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
        if isinstance(current_op, SolubilityOpDescriptor):
            # TODO change the code to talk to an atual soluability 
            self._camera_pub.publish(camera_command=CameraCommand.RECORD)
            rospy.sleep(2)
            self._camera_pub.publish(camera_command=CameraCommand.STOPRECORD)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return True

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}