#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.soluibility_station import SolubilityStation
from archemist.state.state import State
from archemist_msgs.msg import CameraCommand
from enum import Enum

from rospy.core import is_shutdown

class SolubilityStation_Handler:
    def __init__(self):
        rospy.init_node("CameraHandler")
        print("CameraHandler running")
        self.coder = rosMsgCoder()
        self.state = State()
        self.state.initializeState(False)
        self._cameraState = self.state.getStation('SolubilityStation')
        self.pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self.pubCamera = rospy.Publisher("/camera1/commands", CameraCommand, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)
        #rospy.spin()

    def handler_cb(self):
        pass

    def handle(self):
        self.state.updateFromDB()
        self._cameraState = self.state.getStation('SolubilityStation')
        if self._cameraState._assigned_batch is not None:
            current_op = self._cameraState._assigned_batch.getCurrentOp()

            self.pubCamera.publish(camera_command=CameraCommand.RECORD)
            rospy.sleep(current_op.duration)
            self.pubCamera.publish(camera_command=CameraCommand.STOPRECORD)

            self._cameraState._assigned_batch.advanceProcessState()
            self._cameraState._processed_batch = self._cameraState._assigned_batch
            self._cameraState._assigned_batch = None
            self.state.modifyObjectDB(self._cameraState)

if __name__ == '__main__':
    ika_handler = SolubilityStation_Handler()
