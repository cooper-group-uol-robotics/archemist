# !/usr/bin/env python

import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from archemist.core.processing.handler import StationHandler
from roslabware_msgs.msg import FiltrationCmd,FiltrationStatus, BaseValveCmd
from std_msgs.msg import Bool
from rospy.core import is_shutdown

class FiltrationStationROSSender:
    def __init__(self, station: Station):
        super().__init__(station)
        self._Filtration_current_task_complete = False
        rospy.init_node(f'{self._station}_handler')
        self.pubBaseValve = rospy.Publisher("/Optimax_BaseValve_Commands", BaseValveCmd, queue_size=1)
        self.pubFS = rospy.Publisher("/Filtration_Commands", FiltrationCmd, queue_size=1)
        rospy.Subscriber('/filtration/task_complete', Bool, self._fs_state_update, queue_size=1)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            rospy.sleep(1)
               
    def run(self):
        rospy.loginfo('opening base valve')
        for i in range(10):
            self.pubBaseValve.publish(valve_command=BaseValveCmd.OPEN)
        rospy.sleep(3)
        rospy.loginfo('closing base valve')
        for i in range(10):
            self.pubBaseValve.publish(valve_command=BaseValveCmd.CLOSE)
        rospy.sleep(20)
        rospy.loginfo('opening vacuum')
        for i in range(10):
            self.pubFS.publish(filtration_system_command=FiltrationCmd.VACUUM)
        for i in range(10):
            self.pubBaseValve.publish(valve_command=BaseValveCmd.OPEN)
        rospy.sleep(20)
        rospy.loginfo('closing vacuum')
        for i in range(10):
            self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)
        rospy.loginfo('opening drain')
        for i in range(10):
            self.pubFS.publish(filtration_system_command=FiltrationCmd.DRAIN)
        rospy.sleep(20)
        rospy.loginfo('closing drain')
        for i in range(10):
            self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)
        rospy.loginfo('Stopping all')
        for i in range(10):
            self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)
    
    def _fs_state_update(self, msg):
        if msg.data == True:
            self._Filtration_current_task_complete = True
        elif msg.data == False:
            self._Filtration_current_task_complete = False



if __name__ == "__main__":
    sender = FiltrationStationROSSender()
    sender.run()
