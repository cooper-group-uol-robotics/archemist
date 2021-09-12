#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.soluibility_station import SolubilityStation, SolubilityOpDescriptor
from archemist.state.state import State
from archemist_msgs.msg import IKACommand
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
        self.pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)
        #rospy.spin()

    def handle(self):
        self.state.updateFromDB()
        self._cameraState = self.state.getStation('SolubilityStation')
        if self._cameraState._assigned_batch is not None:
            current_op = self._cameraState._assigned_batch.getCurrentOp()
            current_op.addTimeStamp()

            # do something

            self._cameraState._assigned_batch.advanceProcessState()
            self._cameraState._processed_batch = self._cameraState._assigned_batch
            self._cameraState._assigned_batch = None
            self.state.modifyObjectDB(self._cameraState)

